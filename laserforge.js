/* LaserForge 0.1.0 -- SVG -> GRBL gcode (flat + rotary).
 *
 * A faithful JavaScript port of laserforge.py (the Python engine in
 * the parent folder). Same inputs, same gcode out, same report lines,
 * same exit-code discipline: quirks of the Python original are ported
 * on purpose -- parity beats elegance. See PARITY.md for the proof.
 *
 * Zero dependencies. Runs in a browser (window.LaserForge) and under
 * Node (module.exports + a CLI when run as the main module).
 *
 * Pipeline: SVG load (units -> mm, transforms, curves flattened, Y
 * flipped so the burn matches the on-screen design) -> import cleanup
 * guards (duplicate dedupe, auto-join, unclosed-path warnings -- all
 * reported, never silent) -> layers of ordered ops (line / fill /
 * cut) -> gcode in the Falcon/GRBL dialect, self-verified before it
 * is handed back.
 */
(function (global) {
"use strict";

// ---------------------------------------------------------------------------
// Constants (laserforge.py module level)
// ---------------------------------------------------------------------------

var VERSION = "0.1.0";
var FLATTEN_TOL = 0.05;            // mm, curve flattening tolerance
var CLOSE_TOL = 0.01;              // mm: endpoints this close count as closed
var JOIN_TOL = 0.05;               // mm: auto-join / gap-close tolerance
var DEFAULT_BED = [400.0, 415.0];  // mm, Falcon Pro
var RAPID_FEED = 6000.0;           // mm/min, assumed rapid rate for estimates
var IN_MM = 25.4;                  // exact inch -> mm

// ---------------------------------------------------------------------------
// Errors: Python die() prints "laserforge: error: <msg>" and exits with a
// code (2 = bad input, 3 = self-verification failed). Here die() throws an
// LFError carrying the code; the CLI translates it back to print + exit.
// ---------------------------------------------------------------------------

function LFError(msg, code) {
  this.message = msg;
  this.code = (code === undefined) ? 2 : code;
  this.verifyProblems = null; // optional list, printed before the error
}
LFError.prototype = Object.create(Error.prototype);
LFError.prototype.name = "LFError";

function die(msg, code) {
  throw new LFError(msg, code);
}

// ---------------------------------------------------------------------------
// Python-compatible number formatting. The Python engine formats with
// "%.*f" (round-half-even on the exact binary value), "%g", round() and
// str(). JS toFixed() rounds half-up, so the fixed formats go through a
// half-even helper to keep emitted gcode byte-identical.
// ---------------------------------------------------------------------------

function pyRoundHalfEven(v, nd) {
  var p = Math.pow(10, nd);
  var x = v * p;
  var i = Math.floor(x);
  var frac = x - i;
  if (frac > 0.5) {
    i += 1;
  } else if (frac === 0.5 && Math.abs(i % 2) === 1) {
    i += 1;
  }
  return i / p;
}

function pyFixed(v, n) {
  // Python "%.*f" % v -- sign is kept even when the result rounds to 0.
  var neg = v < 0 || Object.is(v, -0);
  var r = pyRoundHalfEven(Math.abs(v), n);
  var s = r.toFixed(n);
  return neg ? "-" + s : s;
}

function fmt(v) {
  // Python fmt(): "%.3f" with "-0.000" mapped to "0.000".
  var s = pyFixed(v, 3);
  return s === "-0.000" ? "0.000" : s;
}

function fmtFeed(v) {
  return Number.isInteger(v) ? String(Math.trunc(v)) : pyFixed(v, 1);
}

function pyG(v) {
  // Python "%g" (6 significant digits, trailing zeros stripped).
  if (v === 0) return "0";
  if (!isFinite(v)) return String(v);
  var neg = v < 0;
  var a = Math.abs(v);
  var m = /^(\d)\.(\d{5})e([+-]\d+)$/.exec(a.toExponential(5));
  if (!m) return String(v);
  var digits = (m[1] + m[2]).replace(/0+$/, "");
  if (digits === "") digits = "0";
  var exp = parseInt(m[3], 10);
  var out;
  if (exp < -4 || exp >= 6) {
    var mant = digits[0] + (digits.length > 1 ? "." + digits.slice(1) : "");
    out = mant + "e" + (exp < 0 ? "-" : "+") +
      String(Math.abs(exp)).padStart(2, "0");
  } else if (exp >= 0) {
    if (digits.length <= exp + 1) {
      out = digits + "0".repeat(exp + 1 - digits.length);
    } else {
      out = digits.slice(0, exp + 1) + "." + digits.slice(exp + 1);
    }
  } else {
    out = "0." + "0".repeat(-exp - 1) + digits;
  }
  return (neg ? "-" : "") + out;
}

function pyStrFloat(v) {
  // Python str(float): shortest round-trip, integral floats keep ".0".
  if (Number.isInteger(v) && Math.abs(v) < 1e16) return v.toFixed(1);
  return String(v);
}

function pyStr(v) {
  if (typeof v === "number") return pyStrFloat(v);
  if (typeof v === "boolean") return v ? "True" : "False";
  if (v === null || v === undefined) return "None";
  return String(v);
}

function pyRepr(v) {
  // Python %r for the strings that appear in die() messages.
  var str = String(v);
  var q = (str.indexOf("'") !== -1 && str.indexOf('"') === -1) ? '"' : "'";
  var out = q;
  for (var i = 0; i < str.length; i++) {
    var ch = str[i];
    if (ch === "\\") out += "\\\\";
    else if (ch === q) out += "\\" + q;
    else if (ch === "\n") out += "\\n";
    else if (ch === "\r") out += "\\r";
    else if (ch === "\t") out += "\\t";
    else out += ch;
  }
  return out + q;
}

function clamp(v, lo, hi) {
  return lo > v ? lo : (v > hi ? hi : v);
}

function dist(a, b) {
  return Math.hypot(a[0] - b[0], a[1] - b[1]);
}

function ptEq(a, b) {
  return a[0] === b[0] && a[1] === b[1];
}

// ---------------------------------------------------------------------------
// 2D affine transforms as [a, b, c, d, e, f]:
//   x' = a*x + c*y + e
//   y' = b*x + d*y + f
// ---------------------------------------------------------------------------

var IDENTITY = [1.0, 0.0, 0.0, 1.0, 0.0, 0.0];

function matMul(m, n) {
  // Return m composed with n (n applied to the point first).
  return [
    m[0] * n[0] + m[2] * n[1],
    m[1] * n[0] + m[3] * n[1],
    m[0] * n[2] + m[2] * n[3],
    m[1] * n[2] + m[3] * n[3],
    m[0] * n[4] + m[2] * n[5] + m[4],
    m[1] * n[4] + m[3] * n[5] + m[5]
  ];
}

function matApply(m, p) {
  return [m[0] * p[0] + m[2] * p[1] + m[4],
          m[1] * p[0] + m[3] * p[1] + m[5]];
}

function matScaleFactor(m) {
  return Math.max(Math.hypot(m[0], m[1]), Math.hypot(m[2], m[3]), 1e-9);
}

function parseTransform(text) {
  // Parse an SVG transform attribute into a matrix.
  var mat = IDENTITY.slice();
  if (!text) return mat;
  var re = /(matrix|translate|scale|rotate|skewX|skewY)\s*\(([^)]*)\)/g;
  var match;
  while ((match = re.exec(text)) !== null) {
    var name = match[1];
    var vals = match[2].trim().split(/[\s,]+/).filter(function (v) {
      return v !== "";
    }).map(function (v) {
      var f = Number(v);
      if (Number.isNaN(f)) throw new Error("bad transform number: " + v);
      return f;
    });
    var t = null;
    if (name === "matrix" && vals.length === 6) {
      t = vals.slice();
    } else if (name === "translate") {
      t = [1.0, 0.0, 0.0, 1.0, vals[0], vals.length > 1 ? vals[1] : 0.0];
    } else if (name === "scale") {
      t = [vals[0], 0.0, 0.0, vals.length > 1 ? vals[1] : vals[0], 0.0, 0.0];
    } else if (name === "rotate") {
      var th = vals[0] * (Math.PI / 180.0);
      var rot = [Math.cos(th), Math.sin(th), -Math.sin(th), Math.cos(th),
                 0.0, 0.0];
      if (vals.length >= 3) {
        var cx = vals[1], cy = vals[2];
        t = matMul(matMul([1.0, 0.0, 0.0, 1.0, cx, cy], rot),
                   [1.0, 0.0, 0.0, 1.0, -cx, -cy]);
      } else {
        t = rot;
      }
    } else if (name === "skewX") {
      t = [1.0, 0.0, Math.tan(vals[0] * (Math.PI / 180.0)), 1.0, 0.0, 0.0];
    } else if (name === "skewY") {
      t = [1.0, Math.tan(vals[0] * (Math.PI / 180.0)), 0.0, 1.0, 0.0, 0.0];
    } else {
      continue;
    }
    mat = matMul(mat, t);
  }
  return mat;
}

// ---------------------------------------------------------------------------
// SVG lengths / units
// ---------------------------------------------------------------------------

var UNIT_TO_MM = {
  "mm": 1.0,
  "cm": 10.0,
  "in": 25.4,
  "pt": 25.4 / 72.0,
  "pc": 25.4 / 6.0,
  "px": 25.4 / 96.0,
  "": 25.4 / 96.0 // bare user units are px at 96 dpi
};

var LENGTH_RE = /^\s*([+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?)\s*([a-z%]*)\s*$/;

function parseLength(text) {
  // Parse an SVG length -> [value, unit] or null.
  if (text === null || text === undefined) return null;
  var m = LENGTH_RE.exec(text);
  if (!m) return null;
  return [parseFloat(m[1]), m[2]];
}

function lengthToMm(text) {
  var parsed = parseLength(text);
  if (parsed === null) return null;
  var value = parsed[0], unit = parsed[1];
  if (unit === "%") return null;
  var f = UNIT_TO_MM[unit];
  return value * (f === undefined ? 25.4 / 96.0 : f);
}

// ---------------------------------------------------------------------------
// Curve flattening (local coordinates; tolerance in the same units)
// ---------------------------------------------------------------------------

function _distToLine(p, a, b) {
  var dx = b[0] - a[0], dy = b[1] - a[1];
  var chord = Math.hypot(dx, dy);
  if (chord < 1e-12) return dist(p, a);
  return Math.abs((p[0] - a[0]) * dy - (p[1] - a[1]) * dx) / chord;
}

function _mid(a, b) {
  return [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0];
}

function flattenCubic(p0, p1, p2, p3, tol, out) {
  // Append flattened points of a cubic Bezier (excluding p0) to out.
  var stack = [[p0, p1, p2, p3, 0]];
  while (stack.length) {
    var seg = stack.pop();
    var a = seg[0], b = seg[1], c = seg[2], d = seg[3], depth = seg[4];
    if (depth >= 20 ||
        Math.max(_distToLine(b, a, d), _distToLine(c, a, d)) <= tol) {
      out.push(d);
    } else {
      var ab = _mid(a, b), bc = _mid(b, c), cd = _mid(c, d);
      var abc = _mid(ab, bc), bcd = _mid(bc, cd);
      var abcd = _mid(abc, bcd);
      stack.push([abcd, bcd, cd, d, depth + 1]);
      stack.push([a, ab, abc, abcd, depth + 1]);
    }
  }
}

function flattenQuad(p0, q, p2, tol, out) {
  // Append flattened points of a quadratic Bezier (excluding p0) to out.
  var b = [p0[0] + (2.0 / 3.0) * (q[0] - p0[0]),
           p0[1] + (2.0 / 3.0) * (q[1] - p0[1])];
  var c = [p2[0] + (2.0 / 3.0) * (q[0] - p2[0]),
           p2[1] + (2.0 / 3.0) * (q[1] - p2[1])];
  flattenCubic(p0, b, c, p2, tol, out);
}

function arcToPoints(p0, rx, ry, rotDeg, largeArc, sweep, p1, tol) {
  // SVG endpoint arc -> points excluding start, including end.
  // Implements the SVG 1.1 F.6.5 endpoint-to-center conversion.
  if (rx === 0 || ry === 0) return [p1];
  if (ptEq(p0, p1)) return [];
  rx = Math.abs(rx);
  ry = Math.abs(ry);
  var phi = rotDeg * (Math.PI / 180.0);
  var cosP = Math.cos(phi), sinP = Math.sin(phi);
  var dx = (p0[0] - p1[0]) / 2.0, dy = (p0[1] - p1[1]) / 2.0;
  var x1p = cosP * dx + sinP * dy;
  var y1p = -sinP * dx + cosP * dy;
  var lam = (x1p * x1p) / (rx * rx) + (y1p * y1p) / (ry * ry);
  if (lam > 1.0) {
    var s = Math.sqrt(lam);
    rx = rx * s;
    ry = ry * s;
  }
  var num = rx * rx * ry * ry - rx * rx * y1p * y1p - ry * ry * x1p * x1p;
  var den = rx * rx * y1p * y1p + ry * ry * x1p * x1p;
  var co = den !== 0 ? Math.sqrt(Math.max(0.0, num / den)) : 0.0;
  if (largeArc === sweep) co = -co;
  var cxp = co * rx * y1p / ry;
  var cyp = -co * ry * x1p / rx;
  var cx = cosP * cxp - sinP * cyp + (p0[0] + p1[0]) / 2.0;
  var cy = sinP * cxp + cosP * cyp + (p0[1] + p1[1]) / 2.0;

  function angle(ux, uy, vx, vy) {
    var d = (ux * vx + uy * vy) / (Math.hypot(ux, uy) * Math.hypot(vx, vy));
    var a = Math.acos(clamp(d, -1.0, 1.0));
    return ux * vy - uy * vx < 0 ? -a : a;
  }

  var theta1 = angle(1.0, 0.0, (x1p - cxp) / rx, (y1p - cyp) / ry);
  var dtheta = angle((x1p - cxp) / rx, (y1p - cyp) / ry,
                     (-x1p - cxp) / rx, (-y1p - cyp) / ry);
  if (!sweep && dtheta > 0) dtheta -= 2.0 * Math.PI;
  else if (sweep && dtheta < 0) dtheta += 2.0 * Math.PI;
  var rMax = Math.max(rx, ry);
  var step = rMax > tol
    ? 2.0 * Math.acos(clamp(1.0 - tol / rMax, -1.0, 1.0))
    : Math.PI / 8.0;
  var n = Math.max(1, Math.ceil(Math.abs(dtheta) / Math.max(step, 1e-6)));
  var pts = [];
  for (var i = 1; i <= n; i++) {
    var t = theta1 + dtheta * i / n;
    var ex = rx * Math.cos(t), ey = ry * Math.sin(t);
    pts.push([cosP * ex - sinP * ey + cx, sinP * ex + cosP * ey + cy]);
  }
  pts[pts.length - 1] = p1;
  return pts;
}

function ellipsePoints(cx, cy, rx, ry, tol) {
  // Full ellipse as a closed polyline (first point repeated at the end).
  var rMax = Math.max(rx, ry);
  if (rMax <= 0) return [];
  var step = rMax > tol
    ? 2.0 * Math.acos(clamp(1.0 - tol / rMax, -1.0, 1.0))
    : Math.PI / 4.0;
  var n = Math.max(8, Math.ceil(2.0 * Math.PI / Math.max(step, 1e-6)));
  var pts = [];
  for (var i = 0; i < n; i++) {
    pts.push([cx + rx * Math.cos(2.0 * Math.PI * i / n),
              cy + ry * Math.sin(2.0 * Math.PI * i / n)]);
  }
  pts.push(pts[0]);
  return pts;
}

// ---------------------------------------------------------------------------
// Minimal XML parser (the Python engine uses ElementTree; this produces
// the same element tree shape for SVG documents: {tag, attrs, children}
// with namespace prefixes stripped from tag names). Malformed input
// throws, which loadSVGText turns into the same die() the Python
// engine raises on a parse error.
// ---------------------------------------------------------------------------

function decodeEntities(s) {
  return s.replace(/&(#x[0-9a-fA-F]+|#\d+|amp|lt|gt|quot|apos);/g,
    function (whole, body) {
      if (body === "amp") return "&";
      if (body === "lt") return "<";
      if (body === "gt") return ">";
      if (body === "quot") return '"';
      if (body === "apos") return "'";
      if (body.charAt(0) === "#") {
        var code = body.charAt(1) === "x"
          ? parseInt(body.slice(2), 16) : parseInt(body.slice(1), 10);
        return String.fromCodePoint(code);
      }
      return whole;
    });
}

function parseXML(text) {
  if (text.charCodeAt(0) === 0xFEFF) text = text.slice(1);
  var i = 0;
  var n = text.length;

  function fail(msg) {
    throw new Error(msg + " (at character " + i + ")");
  }
  function startsWith(s) {
    return text.substr(i, s.length) === s;
  }
  function skipMisc() {
    for (;;) {
      while (i < n && /\s/.test(text[i])) i++;
      if (startsWith("<?")) {
        var e1 = text.indexOf("?>", i);
        if (e1 === -1) fail("unterminated processing instruction");
        i = e1 + 2;
      } else if (startsWith("<!--")) {
        var e2 = text.indexOf("-->", i);
        if (e2 === -1) fail("unterminated comment");
        i = e2 + 3;
      } else if (startsWith("<!DOCTYPE")) {
        var j = i + 9;
        var depth = 0;
        while (j < n) {
          if (text[j] === "[") depth++;
          else if (text[j] === "]") depth--;
          else if (text[j] === ">" && depth === 0) break;
          j++;
        }
        if (j >= n) fail("unterminated doctype");
        i = j + 1;
      } else {
        return;
      }
    }
  }
  function parseName() {
    var m = /^[A-Za-z_][A-Za-z0-9_.:-]*/.exec(text.slice(i));
    if (!m) fail("expected a name");
    i += m[0].length;
    return m[0];
  }
  function localName(name) {
    var cut = Math.max(name.lastIndexOf(":"), name.lastIndexOf("}"));
    return cut === -1 ? name : name.slice(cut + 1);
  }
  function parseElement() {
    // text[i] === '<' and this is not a comment/PI/close tag.
    i++;
    var rawName = parseName();
    var elem = { tag: localName(rawName), attrs: {}, children: [] };
    for (;;) {
      while (i < n && /\s/.test(text[i])) i++;
      if (i >= n) fail("unterminated element <" + rawName + ">");
      if (startsWith("/>")) {
        i += 2;
        return elem;
      }
      if (text[i] === ">") {
        i++;
        break;
      }
      var attrName = parseName();
      while (i < n && /\s/.test(text[i])) i++;
      var attrVal = "";
      if (text[i] === "=") {
        i++;
        while (i < n && /\s/.test(text[i])) i++;
        var quote = text[i];
        if (quote !== '"' && quote !== "'") fail("expected quoted value");
        var end = text.indexOf(quote, i + 1);
        if (end === -1) fail("unterminated attribute value");
        attrVal = decodeEntities(text.slice(i + 1, end));
        i = end + 1;
      }
      elem.attrs[attrName] = attrVal;
    }
    for (;;) {
      if (i >= n) fail("unterminated element <" + rawName + ">");
      if (startsWith("</")) {
        i += 2;
        var closeName = parseName();
        while (i < n && /\s/.test(text[i])) i++;
        if (text[i] !== ">") fail("expected '>' in close tag");
        i++;
        if (closeName !== rawName) {
          fail("mismatched close tag </" + closeName + "> for <" +
               rawName + ">");
        }
        return elem;
      }
      if (startsWith("<!--")) {
        var ec = text.indexOf("-->", i);
        if (ec === -1) fail("unterminated comment");
        i = ec + 3;
      } else if (startsWith("<![CDATA[")) {
        var ed = text.indexOf("]]>", i);
        if (ed === -1) fail("unterminated CDATA section");
        i = ed + 3;
      } else if (startsWith("<?")) {
        var ep = text.indexOf("?>", i);
        if (ep === -1) fail("unterminated processing instruction");
        i = ep + 2;
      } else if (text[i] === "<") {
        elem.children.push(parseElement());
      } else {
        var lt = text.indexOf("<", i);
        i = lt === -1 ? n : lt; // character data: not used by the engine
      }
    }
  }

  skipMisc();
  if (text[i] !== "<" || startsWith("</")) fail("no root element found");
  var root = parseElement();
  return root;
}

// ---------------------------------------------------------------------------
// SVG path data parsing (scanner-based; arc flags read as single digits
// so packed forms like "0 01150 90" tokenise the way the SVG spec
// intends)
// ---------------------------------------------------------------------------

var NUM_RE = /[+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?/y;

function Scanner(text) {
  this.s = text || "";
  this.i = 0;
  this.n = this.s.length;
}
Scanner.prototype.skip = function () {
  while (this.i < this.n && " \t\r\n,".indexOf(this.s[this.i]) !== -1) {
    this.i++;
  }
};
Scanner.prototype.eof = function () {
  this.skip();
  return this.i >= this.n;
};
Scanner.prototype.peekCmd = function () {
  this.skip();
  if (this.i < this.n && /[A-Za-z]/.test(this.s[this.i])) {
    return this.s[this.i];
  }
  return null;
};
Scanner.prototype.readCmd = function () {
  this.skip();
  var c = this.s[this.i];
  this.i++;
  return c;
};
Scanner.prototype.readNum = function () {
  this.skip();
  NUM_RE.lastIndex = this.i;
  var m = NUM_RE.exec(this.s);
  if (!m) throw new Error("expected number at " + this.i + " in path data");
  this.i = NUM_RE.lastIndex;
  return parseFloat(m[0]);
};
Scanner.prototype.readFlag = function () {
  this.skip();
  if (this.i >= this.n || "01".indexOf(this.s[this.i]) === -1) {
    throw new Error("expected arc flag at " + this.i + " in path data");
  }
  var v = parseInt(this.s[this.i], 10);
  this.i++;
  return v;
};

function parsePath(d, tol) {
  // Parse SVG path data -> list of {points, closed} in local coordinates.
  // Closed subpaths include the start point again as their last point.
  var sc = new Scanner(d);
  var subpaths = [];
  var pts = null;
  var cur = [0.0, 0.0];
  var start = [0.0, 0.0];
  var cmd = null;
  var prevCubic = null; // previous cubic's second control point
  var prevQuad = null;  // previous quadratic's control point

  function flush() {
    if (pts) subpaths.push({ points: pts, closed: false });
    pts = null;
  }
  function ensurePts() {
    if (pts === null) pts = [cur];
  }

  while (!sc.eof()) {
    var letter = sc.peekCmd();
    if (letter !== null) {
      cmd = sc.readCmd();
    } else {
      if (cmd === null) throw new Error("path data must start with a command");
      if (cmd === "Z" || cmd === "z") {
        throw new Error("unexpected arguments after Z");
      }
      if (cmd === "M" || cmd === "m") cmd = cmd === "M" ? "L" : "l";
    }
    var upper = cmd.toUpperCase();
    var rel = cmd === cmd.toLowerCase() && cmd !== cmd.toUpperCase();

    if (upper === "Z") {
      if (pts) {
        if (dist(pts[pts.length - 1], start) > 1e-12) pts.push(start);
        subpaths.push({ points: pts, closed: true });
        pts = null;
      }
      cur = start;
      prevCubic = prevQuad = null;
      cmd = null;
      continue;
    }

    if (upper === "M") {
      var mx = sc.readNum(), my = sc.readNum();
      if (rel) { mx += cur[0]; my += cur[1]; }
      flush();
      cur = [mx, my];
      start = cur;
      pts = [cur];
      prevCubic = prevQuad = null;
      cmd = rel ? "l" : "L";
      continue;
    }

    ensurePts();

    if (upper === "L") {
      var lx = sc.readNum(), ly = sc.readNum();
      if (rel) { lx += cur[0]; ly += cur[1]; }
      cur = [lx, ly];
      pts.push(cur);
      prevCubic = prevQuad = null;
    } else if (upper === "H") {
      var hx = sc.readNum();
      if (rel) hx += cur[0];
      cur = [hx, cur[1]];
      pts.push(cur);
      prevCubic = prevQuad = null;
    } else if (upper === "V") {
      var vy = sc.readNum();
      if (rel) vy += cur[1];
      cur = [cur[0], vy];
      pts.push(cur);
      prevCubic = prevQuad = null;
    } else if (upper === "C") {
      var cx1 = sc.readNum(), cy1 = sc.readNum();
      var cx2 = sc.readNum(), cy2 = sc.readNum();
      var cx = sc.readNum(), cy = sc.readNum();
      if (rel) {
        cx1 += cur[0]; cy1 += cur[1];
        cx2 += cur[0]; cy2 += cur[1];
        cx += cur[0]; cy += cur[1];
      }
      var flatC = [];
      flattenCubic(cur, [cx1, cy1], [cx2, cy2], [cx, cy], tol, flatC);
      pts.push.apply(pts, flatC);
      prevCubic = [cx2, cy2];
      prevQuad = null;
      cur = [cx, cy];
    } else if (upper === "S") {
      var sx2 = sc.readNum(), sy2 = sc.readNum();
      var sx = sc.readNum(), sy = sc.readNum();
      if (rel) {
        sx2 += cur[0]; sy2 += cur[1];
        sx += cur[0]; sy += cur[1];
      }
      var sx1, sy1;
      if (prevCubic !== null) {
        sx1 = 2.0 * cur[0] - prevCubic[0];
        sy1 = 2.0 * cur[1] - prevCubic[1];
      } else {
        sx1 = cur[0]; sy1 = cur[1];
      }
      var flatS = [];
      flattenCubic(cur, [sx1, sy1], [sx2, sy2], [sx, sy], tol, flatS);
      pts.push.apply(pts, flatS);
      prevCubic = [sx2, sy2];
      prevQuad = null;
      cur = [sx, sy];
    } else if (upper === "Q") {
      var qx1 = sc.readNum(), qy1 = sc.readNum();
      var qx = sc.readNum(), qy = sc.readNum();
      if (rel) {
        qx1 += cur[0]; qy1 += cur[1];
        qx += cur[0]; qy += cur[1];
      }
      var flatQ = [];
      flattenQuad(cur, [qx1, qy1], [qx, qy], tol, flatQ);
      pts.push.apply(pts, flatQ);
      prevCubic = null;
      prevQuad = [qx1, qy1];
      cur = [qx, qy];
    } else if (upper === "T") {
      var tx = sc.readNum(), ty = sc.readNum();
      if (rel) { tx += cur[0]; ty += cur[1]; }
      var tx1, ty1;
      if (prevQuad !== null) {
        tx1 = 2.0 * cur[0] - prevQuad[0];
        ty1 = 2.0 * cur[1] - prevQuad[1];
      } else {
        tx1 = cur[0]; ty1 = cur[1];
      }
      var flatT = [];
      flattenQuad(cur, [tx1, ty1], [tx, ty], tol, flatT);
      pts.push.apply(pts, flatT);
      prevCubic = null;
      prevQuad = [tx1, ty1];
      cur = [tx, ty];
    } else if (upper === "A") {
      var arx = sc.readNum(), ary = sc.readNum();
      var arot = sc.readNum();
      var alarge = sc.readFlag();
      var asweep = sc.readFlag();
      var ax = sc.readNum(), ay = sc.readNum();
      if (rel) { ax += cur[0]; ay += cur[1]; }
      pts.push.apply(pts, arcToPoints(cur, arx, ary, arot, alarge, asweep,
                                       [ax, ay], tol));
      cur = [ax, ay];
      prevCubic = prevQuad = null;
    } else {
      throw new Error("unsupported path command " + pyRepr(cmd));
    }
  }

  flush();
  return subpaths;
}

// ---------------------------------------------------------------------------
// SVG shape elements -> {points, closed} in local coordinates
// ---------------------------------------------------------------------------

function attrNum(elem, name, dflt) {
  if (dflt === undefined) dflt = 0.0;
  var parsed = parseLength(elem.attrs[name]);
  return parsed ? parsed[0] : dflt;
}

function rectSubpaths(elem, tol) {
  var x = attrNum(elem, "x"), y = attrNum(elem, "y");
  var w = attrNum(elem, "width"), h = attrNum(elem, "height");
  if (w <= 0 || h <= 0) return [];
  var hasRx = elem.attrs.rx !== undefined;
  var hasRy = elem.attrs.ry !== undefined;
  if (!hasRx && !hasRy) {
    return [{ points: [[x, y], [x + w, y], [x + w, y + h], [x, y + h],
                       [x, y]], closed: true }];
  }
  var rxv = hasRx ? attrNum(elem, "rx") : attrNum(elem, "ry");
  var ryv = hasRy ? attrNum(elem, "ry") : rxv;
  rxv = Math.min(rxv, w / 2.0);
  ryv = Math.min(ryv, h / 2.0);
  var pts = [[x + rxv, y], [x + w - rxv, y]];
  pts.push.apply(pts, arcToPoints(pts[pts.length - 1], rxv, ryv, 0, 0, 1,
                                   [x + w, y + ryv], tol));
  pts.push([x + w, y + h - ryv]);
  pts.push.apply(pts, arcToPoints(pts[pts.length - 1], rxv, ryv, 0, 0, 1,
                                   [x + w - rxv, y + h], tol));
  pts.push([x + rxv, y + h]);
  pts.push.apply(pts, arcToPoints(pts[pts.length - 1], rxv, ryv, 0, 0, 1,
                                   [x, y + h - ryv], tol));
  pts.push([x, y + ryv]);
  pts.push.apply(pts, arcToPoints(pts[pts.length - 1], rxv, ryv, 0, 0, 1,
                                   [x + rxv, y], tol));
  return [{ points: pts, closed: true }];
}

function pointsAttr(elem) {
  var raw = elem.attrs.points || "";
  var trimmed = raw.trim();
  var vals = trimmed === "" ? [] : trimmed.split(/[\s,]+/).filter(
    function (v) { return v !== ""; }).map(function (v) {
      var f = Number(v);
      if (Number.isNaN(f)) throw new Error("bad points number: " + v);
      return f;
    });
  var pts = [];
  for (var i = 0; i + 1 < vals.length; i += 2) pts.push([vals[i], vals[i + 1]]);
  return pts;
}

function shapeSubpaths(elem, tag, tol) {
  if (tag === "path") return parsePath(elem.attrs.d || "", tol);
  if (tag === "rect") return rectSubpaths(elem, tol);
  if (tag === "circle") {
    var cpts = ellipsePoints(attrNum(elem, "cx"), attrNum(elem, "cy"),
                             attrNum(elem, "r"), attrNum(elem, "r"), tol);
    return cpts.length ? [{ points: cpts, closed: true }] : [];
  }
  if (tag === "ellipse") {
    var epts = ellipsePoints(attrNum(elem, "cx"), attrNum(elem, "cy"),
                             attrNum(elem, "rx"), attrNum(elem, "ry"), tol);
    return epts.length ? [{ points: epts, closed: true }] : [];
  }
  if (tag === "line") {
    return [{ points: [[attrNum(elem, "x1"), attrNum(elem, "y1")],
                       [attrNum(elem, "x2"), attrNum(elem, "y2")]],
              closed: false }];
  }
  if (tag === "polyline") {
    var lpts = pointsAttr(elem);
    return lpts.length >= 2 ? [{ points: lpts, closed: false }] : [];
  }
  if (tag === "polygon") {
    var gpts = pointsAttr(elem);
    if (gpts.length >= 2) {
      if (!ptEq(gpts[0], gpts[gpts.length - 1])) gpts = gpts.concat([gpts[0]]);
      return [{ points: gpts, closed: true }];
    }
    return [];
  }
  return [];
}

var SHAPE_TAGS = { path: 1, rect: 1, circle: 1, ellipse: 1, line: 1,
                   polyline: 1, polygon: 1 };
var SKIP_TAGS = { defs: 1, title: 1, desc: 1, metadata: 1, style: 1,
                  script: 1 };

function isHidden(elem) {
  if (elem.attrs.display === "none" || elem.attrs.visibility === "hidden") {
    return true;
  }
  var style = (elem.attrs.style || "").replace(/ /g, "");
  return style.indexOf("display:none") !== -1 ||
         style.indexOf("visibility:hidden") !== -1;
}

// ---------------------------------------------------------------------------
// SVG loading: shapes in document mm, Y flipped (bed origin lower-left)
// ---------------------------------------------------------------------------

function Shape(points, closed, label, index, elemId, groupIds) {
  this.points = points;
  this.closed = closed;
  this.label = label;
  this.index = index;
  this.elemId = elemId;
  this.groupIds = groupIds;
}

function rootTransform(root) {
  // -> {mat: matrix mapping user units -> mm, docH: document height mm
  // or null}
  var vb = null;
  if (root.attrs.viewBox) {
    var trimmed = root.attrs.viewBox.trim();
    var parts = trimmed === "" ? [] : trimmed.split(/[\s,]+/).filter(
      function (v) { return v !== ""; }).map(Number);
    if (parts.length === 4) vb = parts;
  }
  var wMm = lengthToMm(root.attrs.width);
  var hMm = lengthToMm(root.attrs.height);
  if (vb === null) {
    // No viewBox: user units are px at 96 dpi; the document height in
    // mm comes from the height attribute if it carries one.
    var s0 = 25.4 / 96.0;
    return { mat: [s0, 0.0, 0.0, s0, 0.0, 0.0], docH: hMm };
  }
  var minx = vb[0], miny = vb[1], vbw = vb[2], vbh = vb[3];
  if (wMm === null) wMm = vbw * 25.4 / 96.0;
  if (hMm === null) hMm = vbh * 25.4 / 96.0;
  if (vbw <= 0 || vbh <= 0) return { mat: IDENTITY.slice(), docH: hMm };
  var par = (root.attrs.preserveAspectRatio || "xMidYMid meet").trim();
  if (par === "none") {
    var sx = wMm / vbw, sy = hMm / vbh;
    return { mat: [sx, 0.0, 0.0, sy, -minx * sx, -miny * sy], docH: hMm };
  }
  var s = par.indexOf("meet") !== -1
    ? Math.min(wMm / vbw, hMm / vbh) : Math.max(wMm / vbw, hMm / vbh);
  var ax = par.indexOf("xMid") !== -1 ? (wMm - vbw * s) / 2.0
    : (par.indexOf("xMax") !== -1 ? wMm - vbw * s : 0.0);
  var ay = par.indexOf("YMid") !== -1 ? (hMm - vbh * s) / 2.0
    : (par.indexOf("YMax") !== -1 ? hMm - vbh * s : 0.0);
  return { mat: [s, 0.0, 0.0, s, ax - minx * s, ay - miny * s], docH: hMm };
}

function loadSVGText(text, sourceName, inputLabel) {
  // Load an SVG -> {shapes, report}. Shapes are in document mm, Y
  // flipped so the burn matches the on-screen design. sourceName is
  // the basename (report + gcode header); inputLabel is the input
  // path as given (used in error messages, like the Python engine).
  if (inputLabel === undefined) inputLabel = sourceName;
  var root;
  try {
    root = parseXML(text);
  } catch (exc) {
    die("cannot parse " + inputLabel + " as XML: " + exc.message);
  }
  if (root.tag !== "svg") die(inputLabel + ": root element is not <svg>");
  var rt = rootTransform(root);
  var base = rt.mat, docH = rt.docH;
  var shapes = [];
  var dropped = 0;

  function add(pts, closed, elem, groupIds) {
    var clean = [];
    for (var i = 0; i < pts.length; i++) {
      var p = pts[i];
      if (!clean.length || dist(p, clean[clean.length - 1]) > 1e-9) {
        clean.push(p);
      }
    }
    var total = 0;
    for (var j = 0; j + 1 < clean.length; j++) {
      total += dist(clean[j], clean[j + 1]);
    }
    if (clean.length < 2 || total <= 0) {
      dropped++;
      return;
    }
    if (!closed && clean.length >= 3 &&
        dist(clean[0], clean[clean.length - 1]) <= CLOSE_TOL) {
      clean[clean.length - 1] = clean[0];
      closed = true;
    }
    var idx = shapes.length;
    var elemId = (elem.attrs.id !== undefined) ? elem.attrs.id : null;
    shapes.push(new Shape(clean, closed,
                          elemId || "shape-" + (idx + 1),
                          idx, elemId, groupIds.slice()));
  }

  function walk(elem, mat, groupIds) {
    var tag = elem.tag;
    if (SKIP_TAGS[tag] || isHidden(elem)) return;
    var m = matMul(mat, parseTransform(elem.attrs.transform));
    if (SHAPE_TAGS[tag]) {
      var tolLocal = FLATTEN_TOL / matScaleFactor(m);
      var subs;
      try {
        subs = shapeSubpaths(elem, tag, tolLocal);
      } catch (exc) {
        die(inputLabel + ": <" + tag + "> " + (elem.attrs.id || "") +
            ": " + exc.message);
      }
      for (var i = 0; i < subs.length; i++) {
        add(subs[i].points.map(function (p) { return matApply(m, p); }),
            subs[i].closed, elem, groupIds);
      }
      return;
    }
    var childGroups = elem.attrs.id !== undefined
      ? groupIds.concat([elem.attrs.id]) : groupIds;
    for (var c = 0; c < elem.children.length; c++) {
      walk(elem.children[c], m, childGroups);
    }
  }

  walk(root, base, []);

  // Flip Y: SVG is Y-down, the bed is Y-up. Flip around the document
  // height when known, else around the shapes' own top edge.
  if (shapes.length) {
    var flipRef;
    if (docH !== null) {
      flipRef = docH;
    } else {
      flipRef = -Infinity;
      shapes.forEach(function (sh) {
        sh.points.forEach(function (p) {
          if (p[1] > flipRef) flipRef = p[1];
        });
      });
    }
    shapes.forEach(function (sh) {
      sh.points = sh.points.map(function (p) { return [p[0], flipRef - p[1]]; });
    });
  }

  var report = ["import: loaded " + shapes.length + " shape(s) from " +
                sourceName];
  if (dropped) {
    report.push("import: dropped " + dropped + " degenerate shape(s) " +
                "(fewer than 2 distinct points / zero length)");
  }
  return { shapes: shapes, report: report };
}

// ---------------------------------------------------------------------------
// Import cleanup guards -- every action is reported, never silent
// ---------------------------------------------------------------------------

function cmpPtSeq(a, b) {
  var n = Math.min(a.length, b.length);
  for (var i = 0; i < n; i++) {
    if (a[i][0] !== b[i][0]) return a[i][0] < b[i][0] ? -1 : 1;
    if (a[i][1] !== b[i][1]) return a[i][1] < b[i][1] ? -1 : 1;
  }
  return a.length === b.length ? 0 : (a.length < b.length ? -1 : 1);
}

function canonicalKey(sh) {
  var pts = sh.points.map(function (p) {
    return [pyRoundHalfEven(p[0], 2), pyRoundHalfEven(p[1], 2)];
  });
  if (sh.closed && pts.length > 1 && ptEq(pts[0], pts[pts.length - 1])) {
    pts = pts.slice(0, pts.length - 1);
  }
  function ser(seq) {
    return JSON.stringify(seq.map(function (p) {
      return [p[0] === 0 ? 0 : p[0], p[1] === 0 ? 0 : p[1]];
    }));
  }
  if (sh.closed && pts.length >= 2 && pts.length <= 400) {
    var best = null;
    var seqs = [pts, pts.slice().reverse()];
    for (var s = 0; s < seqs.length; s++) {
      var seq = seqs[s];
      for (var k = 0; k < seq.length; k++) {
        var rot = seq.slice(k).concat(seq.slice(0, k));
        if (best === null || cmpPtSeq(rot, best) < 0) best = rot;
      }
    }
    return "C:" + ser(best);
  }
  var fwd = pts, rev = pts.slice().reverse();
  return (sh.closed ? "C:" : "O:") + ser(cmpPtSeq(fwd, rev) <= 0 ? fwd : rev);
}

function tryJoin(a, b) {
  // If an endpoint of a is within JOIN_TOL of an endpoint of b, return
  // {points, gap}; else null.
  var combos = [
    [a.points[a.points.length - 1], b.points[0],
     function () { return a.points.concat(b.points); }],
    [a.points[a.points.length - 1], b.points[b.points.length - 1],
     function () { return a.points.concat(b.points.slice().reverse()); }],
    [a.points[0], b.points[b.points.length - 1],
     function () { return b.points.concat(a.points); }],
    [a.points[0], b.points[0],
     function () { return b.points.slice().reverse().concat(a.points); }]
  ];
  for (var i = 0; i < combos.length; i++) {
    var gap = dist(combos[i][0], combos[i][1]);
    if (gap <= JOIN_TOL) {
      var merged = combos[i][2]();
      var clean = [];
      for (var j = 0; j < merged.length; j++) {
        if (!clean.length || dist(merged[j], clean[clean.length - 1]) > 1e-9) {
          clean.push(merged[j]);
        }
      }
      return { points: clean, gap: gap };
    }
  }
  return null;
}

function cleanupShapes(shapes, report) {
  // 1) duplicate-path dedupe
  var seen = new Map();
  var kept = [];
  shapes.forEach(function (sh) {
    var key = canonicalKey(sh);
    if (seen.has(key)) {
      report.push("import: removed duplicate path '" + sh.label +
                  "' (identical geometry to '" + seen.get(key) + "')");
    } else {
      seen.set(key, sh.label);
      kept.push(sh);
    }
  });
  shapes = kept;

  // 2) auto-join touching segments + close small gaps (loop: a join
  //    can create a shape whose own ends now meet)
  var changed = true;
  while (changed) {
    changed = false;
    for (var s = 0; s < shapes.length; s++) {
      var sh0 = shapes[s];
      if (!sh0.closed && sh0.points.length >= 3) {
        var gap0 = dist(sh0.points[0], sh0.points[sh0.points.length - 1]);
        if (gap0 <= JOIN_TOL) {
          sh0.points[sh0.points.length - 1] = sh0.points[0];
          sh0.closed = true;
          report.push("import: closed path '" + sh0.label +
                      "' (endpoints " + fmt(gap0) + " mm apart)");
          changed = true;
        }
      }
    }
    for (var i = 0; i < shapes.length; i++) {
      if (shapes[i].closed) continue;
      var mergedDone = false;
      for (var j = i + 1; j < shapes.length; j++) {
        if (shapes[j].closed) continue;
        var res = tryJoin(shapes[i], shapes[j]);
        if (res) {
          report.push("import: auto-joined '" + shapes[i].label + "' + '" +
                      shapes[j].label + "' (endpoints " + fmt(res.gap) +
                      " mm apart)");
          shapes[i].points = res.points;
          shapes[i].label = shapes[i].label + "+" + shapes[j].label;
          shapes.splice(j, 1);
          changed = true;
          mergedDone = true;
          break;
        }
      }
      if (mergedDone) break;
    }
  }

  // 3) unclosed-path warnings
  shapes.forEach(function (sh) {
    if (!sh.closed) {
      report.push("import: WARNING path '" + sh.label + "' is not closed " +
                  "-- usable for line/cut, excluded from fill");
    }
  });
  return shapes;
}

// ---------------------------------------------------------------------------
// Hatch fill (even-odd across all of a layer's closed shapes together,
// so a shape inside a shape leaves the ring between them filled)
// ---------------------------------------------------------------------------

function hatchSegments(polys, spacing, angleDeg, bidirectional) {
  // Return a list of [[x0, y0], [x1, y1]] hatch segments. Scanlines sit
  // at spacing/2 offsets from the lowest edge; with bidirectional
  // scanning each line's segments run in the opposite direction to the
  // previous line's (serpentine, less travel).
  if (bidirectional === undefined) bidirectional = true;
  if (spacing <= 0) die("fill spacing must be > 0");
  var ang = angleDeg * (Math.PI / 180.0);
  var ca = Math.cos(ang), sa = Math.sin(ang);

  function rot(p) { // rotate by -angle: scan direction becomes +X
    return [ca * p[0] + sa * p[1], -sa * p[0] + ca * p[1]];
  }
  function unrot(p) { // rotate back by +angle
    return [ca * p[0] - sa * p[1], sa * p[0] + ca * p[1]];
  }

  var edges = [];
  var ys = [];
  polys.forEach(function (poly) {
    var rp = poly.map(rot);
    rp.forEach(function (p) { ys.push(p[1]); });
    for (var i = 0; i < rp.length; i++) {
      var x0 = rp[i][0], y0 = rp[i][1];
      var x1 = rp[(i + 1) % rp.length][0], y1 = rp[(i + 1) % rp.length][1];
      if (y0 !== y1) edges.push([x0, y0, x1, y1]);
    }
  });
  if (!edges.length) return [];
  var ymin = Math.min.apply(null, ys), ymax = Math.max.apply(null, ys);
  var segments = [];
  var k = 0;
  var y = ymin + spacing * 0.5;
  while (y < ymax - 1e-9) {
    var xs = [];
    for (var e = 0; e < edges.length; e++) {
      var ed = edges[e];
      if ((ed[1] <= y && y < ed[3]) || (ed[3] <= y && y < ed[1])) {
        xs.push(ed[0] + (y - ed[1]) * (ed[2] - ed[0]) / (ed[3] - ed[1]));
      }
    }
    xs.sort(function (a, b) { return a - b; });
    var lineSegs = [];
    for (var m = 0; m + 1 < xs.length; m += 2) {
      if (xs[m + 1] - xs[m] > 1e-9) {
        lineSegs.push([unrot([xs[m], y]), unrot([xs[m + 1], y])]);
      }
    }
    if (bidirectional && k % 2 === 1) {
      lineSegs = lineSegs.slice().reverse().map(function (sg) {
        return [sg[1], sg[0]];
      });
    }
    segments.push.apply(segments, lineSegs);
    k += 1;
    y += spacing;
  }
  return segments;
}

// ---------------------------------------------------------------------------
// Job model: layers of ordered ops (sub-layers)
// ---------------------------------------------------------------------------

var OP_DEFAULTS = {
  line: { speed: 1000.0, power: 50.0, passes: 1 },
  cut: { speed: 300.0, power: 90.0, passes: 1 },
  fill: { speed: 1500.0, power: 50.0, passes: 1,
          spacing: 0.1, angle: 0.0, bidirectional: true, outline: false }
};
var OP_KEYS = ["speed", "power", "passes", "spacing", "angle",
               "bidirectional", "outline"];

function normalizeOp(raw, layerName) {
  var otype = raw.op || raw.mode || "line";
  if (!OP_DEFAULTS.hasOwnProperty(otype)) {
    die("layer '" + layerName + "': unknown op " + pyRepr(otype) +
        " (want line, fill or cut)");
  }
  var op = {};
  var defaults = OP_DEFAULTS[otype];
  Object.keys(defaults).forEach(function (k) { op[k] = defaults[k]; });
  OP_KEYS.forEach(function (key) {
    if (key in raw) op[key] = raw[key];
  });
  op.type = otype;
  op.speed = Number(op.speed);
  op.power = Number(op.power);
  op.passes = Math.trunc(Number(op.passes));
  if (op.speed <= 0) die("layer '" + layerName + "': speed must be > 0");
  if (!(0.0 <= op.power && op.power <= 100.0)) {
    die("layer '" + layerName + "': power must be between 0 and 100 " +
        "(got " + pyStr(op.power) + ")");
  }
  if (op.passes < 1) die("layer '" + layerName + "': passes must be >= 1");
  if (otype === "fill") {
    op.spacing = Number(op.spacing);
    op.angle = Number(op.angle);
    if (op.spacing <= 0) {
      die("layer '" + layerName + "': fill spacing must be > 0");
    }
  }
  return op;
}

function normalizeLayer(raw, idx) {
  var name = raw.name || "layer-" + (idx + 1);
  var ops;
  if ("ops" in raw) {
    if (!raw.ops.length) die("layer '" + name + "': ops list is empty");
    ops = raw.ops.map(function (o) { return normalizeOp(o, name); });
  } else {
    ops = [normalizeOp(raw, name)];
  }
  return {
    name: name,
    output: ("output" in raw) ? Boolean(raw.output) : true,
    select: ("select" in raw) ? raw.select : null,
    ops: ops
  };
}

function resolveSelect(sel, shapes, claimed) {
  // Geometry assignment (see README): a layer's `select` list names
  // shapes by element id, group id, or label ("shape-<n>"), or by
  // 0-based document index (integer). A layer with no `select` claims
  // every shape not claimed by an earlier layer.
  if (sel === null || sel === undefined) {
    return shapes.filter(function (sh) { return !claimed.has(sh.index); });
  }
  if (!Array.isArray(sel)) sel = [sel];
  var out = [];
  sel.forEach(function (item) {
    var match;
    if (typeof item === "number" && Number.isInteger(item)) {
      match = shapes.filter(function (sh) { return sh.index === item; });
    } else {
      var want = String(item);
      match = shapes.filter(function (sh) {
        return sh.label === want || sh.elemId === want ||
               sh.groupIds.indexOf(want) !== -1;
      });
    }
    match.forEach(function (sh) {
      if (out.indexOf(sh) === -1) out.push(sh);
    });
  });
  return out;
}

function Stroke(points, feed, s, power, layer, op, passNo, passes) {
  this.points = points;
  this.feed = feed;
  this.s = s;
  this.power = power;
  this.layer = layer;
  this.op = op;
  this.passNo = passNo;
  this.passes = passes;
}

function buildStrokes(job, shapes, report) {
  var strokes = [];
  var claimed = new Set();
  job.layers.forEach(function (layer) {
    if (!layer.output) {
      report.push("layer '" + layer.name + "': output disabled -- skipped");
      return;
    }
    var sel = resolveSelect(layer.select, shapes, claimed);
    if (!sel.length) {
      report.push("layer '" + layer.name +
                  "': WARNING no geometry selected -- skipped");
      return;
    }
    sel.forEach(function (sh) { claimed.add(sh.index); });
    report.push("layer '" + layer.name + "': " + sel.length +
                " shape(s), " + layer.ops.length + " op(s)");
    layer.ops.forEach(function (op) {
      var sval = pyRoundHalfEven(op.power * 10.0, 0);
      for (var pno = 1; pno <= op.passes; pno++) {
        if (op.type === "line" || op.type === "cut") {
          sel.forEach(function (sh) {
            var pts = sh.points.slice();
            if (sh.closed && !ptEq(pts[0], pts[pts.length - 1])) {
              pts.push(pts[0]);
            }
            strokes.push(new Stroke(pts, op.speed, sval, op.power,
                                    layer.name, op.type, pno, op.passes));
          });
        } else { // fill
          var closedShapes = sel.filter(function (sh) { return sh.closed; });
          var nOpen = sel.length - closedShapes.length;
          if (nOpen) {
            report.push("layer '" + layer.name + "': fill skipped " +
                        nOpen + " open path(s) (fill needs closed paths)");
          }
          var polys = [];
          closedShapes.forEach(function (sh) {
            var pts = sh.points.slice();
            if (pts.length && ptEq(pts[0], pts[pts.length - 1])) {
              pts = pts.slice(0, pts.length - 1);
            }
            if (pts.length >= 3) polys.push(pts);
          });
          if (polys.length) {
            hatchSegments(polys, op.spacing, op.angle, op.bidirectional)
              .forEach(function (sg) {
                strokes.push(new Stroke([sg[0], sg[1]], op.speed, sval,
                                        op.power, layer.name, op.type,
                                        pno, op.passes));
              });
            if (op.outline) {
              polys.forEach(function (pts) {
                strokes.push(new Stroke(pts.concat([pts[0]]), op.speed,
                                        sval, op.power, layer.name,
                                        op.type, pno, op.passes));
              });
            }
          }
        }
      }
    });
  });
  return strokes;
}

// ---------------------------------------------------------------------------
// Job statistics, rendering, self-verification.
//
// Time estimate model (also in README): burn time = sum over burn
// segments of length / that op's feed rate; travel time = sum over
// travel moves of length / RAPID_FEED. No acceleration model -- it is
// an estimate, and it errs on the low side for hatch-heavy jobs.
// ---------------------------------------------------------------------------

function analyze(strokes) {
  var burn = 0.0, travel = 0.0, minutes = 0.0;
  var pos = [0.0, 0.0];
  var bounds = null;

  function grow(p) {
    if (bounds === null) {
      bounds = [p[0], p[1], p[0], p[1]];
    } else {
      bounds[0] = Math.min(bounds[0], p[0]);
      bounds[1] = Math.min(bounds[1], p[1]);
      bounds[2] = Math.max(bounds[2], p[0]);
      bounds[3] = Math.max(bounds[3], p[1]);
    }
  }

  strokes.forEach(function (st) {
    var d = dist(pos, st.points[0]);
    travel += d;
    minutes += d / RAPID_FEED;
    var prev = st.points[0];
    grow(prev);
    for (var i = 1; i < st.points.length; i++) {
      var p = st.points[i];
      var seg = dist(prev, p);
      burn += seg;
      minutes += seg / st.feed;
      grow(p);
      prev = p;
    }
    pos = st.points[st.points.length - 1];
  });
  var dHome = dist(pos, [0.0, 0.0]); // final return home (bed origin, or
  // the start point in current mode -- the frame origin in both cases)
  travel += dHome;
  minutes += dHome / RAPID_FEED;
  return { burn: burn, travel: travel, minutes: minutes, bounds: bounds };
}

function rotaryHeaderLines(meta) {
  // Extra header lines for rotary jobs (empty for flat work).
  var r = meta.rotary;
  if (!r) return [];
  var lines = [
    "; Rotary profile: " + r.profile_key + " (" + r.profile_name + ")",
    "; mm per rotation: " + fmt(r.mm_per_rotation) + " mm",
    "; Profile status: " + r.status,
    "; Circumference: " + fmt(r.circumference_mm) + " mm (" +
      fmt(r.circumference_mm / IN_MM) + " in)",
    "; Wrap scale: Y x " + pyFixed(r.factor, 6) +
      " (mm_per_rotation / circumference)"
  ];
  if (meta.calibration) {
    var c = meta.calibration;
    lines.push("; Calibration line: intended wrap arc " +
      fmt(c.test_arc_mm) + " mm (" + fmt(c.test_arc_mm / IN_MM) +
      " in) at mm_per_rotation " + fmt(r.mm_per_rotation) +
      " -- burn on tape, measure the arc, then run --calibrate-apply");
  }
  return lines;
}

function layerComment(st) {
  return "; Layer '" + st.layer + "' -- " + st.op + ", " +
    fmtFeed(st.feed) + " mm/min, " + pyG(st.power) + "% (S" + st.s +
    "), pass " + st.passNo + "/" + st.passes;
}

function renderGcode(strokes, stats, meta) {
  var b = stats.bounds;
  var lines = [
    "; LaserForge " + VERSION + " -- SVG -> GRBL gcode",
    "; Source: " + meta.source,
    "; Mode: " + meta.mode,
    "; Bounds: X" + fmt(b[0]) + " Y" + fmt(b[1]) + " to X" + fmt(b[2]) +
      " Y" + fmt(b[3]),
    "; Burn distance: " + pyFixed(stats.burn, 1) + " mm",
    "; Travel distance: " + pyFixed(stats.travel, 1) + " mm",
    "; Estimated time: " + pyFixed(stats.minutes, 2) + " min",
    "; Layers: " + meta.layer_names.join(", "),
    "G21",
    "G90",
    "M4",
    "S0"
  ];
  lines.splice.apply(lines, [3, 0].concat(rotaryHeaderLines(meta)));
  var group = null;
  strokes.forEach(function (st) {
    var key = st.layer + "\x00" + st.op + "\x00" + st.passNo;
    if (key !== group) {
      group = key;
      lines.push(layerComment(st));
    }
    lines.push("G0 X" + fmt(st.points[0][0]) + " Y" + fmt(st.points[0][1]));
    var first = true;
    var prev = st.points[0];
    for (var i = 1; i < st.points.length; i++) {
      var p = st.points[i];
      if (dist(prev, p) < 1e-12) continue;
      if (first) {
        lines.push("G1 X" + fmt(p[0]) + " Y" + fmt(p[1]) + " F" +
                   fmtFeed(st.feed) + " S" + st.s);
        first = false;
      } else {
        lines.push("G1 X" + fmt(p[0]) + " Y" + fmt(p[1]));
      }
      prev = p;
    }
    lines.push("S0");
  });
  lines.push("; End of job", "S0", "M5", "G0 X0.000 Y0.000");
  return lines.join("\n") + "\n";
}

function verifyGcode(text, bedW, bedH) {
  // Re-read the generated output: recompute the burn bounds from the
  // G1 moves made while S > 0, and flag any emitted position (burn or
  // travel) that leaves the bed.
  var x = 0.0, y = 0.0, s = 0;
  var bounds = null;
  var problems = [];
  text.split("\n").forEach(function (raw) {
    var line = raw.trim();
    if (!line || line.charAt(0) === ";") return;
    var cmd = null;
    var nx = x, ny = y, ns = s;
    line.split(/\s+/).forEach(function (tok) {
      var head = tok.charAt(0);
      if (head === "G") cmd = tok;
      else if (head === "X") nx = parseFloat(tok.slice(1));
      else if (head === "Y") ny = parseFloat(tok.slice(1));
      else if (head === "S") ns = Math.trunc(parseFloat(tok.slice(1)));
    });
    if (cmd === "G0" || cmd === "G1") {
      if (cmd === "G1" && ns > 0) {
        [[x, y], [nx, ny]].forEach(function (p) {
          if (bounds === null) {
            bounds = [p[0], p[1], p[0], p[1]];
          } else {
            bounds[0] = Math.min(bounds[0], p[0]);
            bounds[1] = Math.min(bounds[1], p[1]);
            bounds[2] = Math.max(bounds[2], p[0]);
            bounds[3] = Math.max(bounds[3], p[1]);
          }
        });
      }
      x = nx;
      y = ny;
      if (x < -0.001 || y < -0.001 || x > bedW + 0.001 || y > bedH + 0.001) {
        problems.push("position X" + fmt(x) + " Y" + fmt(y) +
                      " leaves the bed (" + pyFixed(bedW, 0) + " x " +
                      pyFixed(bedH, 0) + " mm)");
      }
    }
    s = ns;
  });
  return { bounds: bounds, problems: problems };
}

// ---------------------------------------------------------------------------
// Current-position start mode: G91 relative emission anchored to the head
// ---------------------------------------------------------------------------

function r3(v) {
  // Round to the 3-decimal grid the gcode text uses; kill -0.0.
  var r = pyRoundHalfEven(v, 3);
  return r === 0 ? 0.0 : r;
}

function strokesBounds(strokes) {
  var b = null;
  strokes.forEach(function (st) {
    st.points.forEach(function (p) {
      if (b === null) {
        b = [p[0], p[1], p[0], p[1]];
      } else {
        b[0] = Math.min(b[0], p[0]);
        b[1] = Math.min(b[1], p[1]);
        b[2] = Math.max(b[2], p[0]);
        b[3] = Math.max(b[3], p[1]);
      }
    });
  });
  return b;
}

function anchorReference(bounds, anchorX, anchorY) {
  // The design point that sits on the head, per the anchor grid.
  // Named in design (screen) terms: left/right are the design's
  // min/max X; bottom is the design's bottom edge, top its top edge.
  // The loader's Y flip is already resolved at this point.
  var rx = { left: bounds[0],
             center: (bounds[0] + bounds[2]) / 2.0,
             right: bounds[2] }[anchorX];
  var ry = { bottom: bounds[1],
             center: (bounds[1] + bounds[3]) / 2.0,
             top: bounds[3] }[anchorY];
  return [rx, ry];
}

function renderGcodeCurrent(strokes, stats, meta) {
  // Current-position start mode: G91 relative moves in a frame whose
  // origin is wherever the head sits when the job starts. The strokes
  // have already been translated so the anchor point is the frame
  // origin. Deltas are computed between successive rounded frame
  // positions, so they telescope exactly on the file's 0.001 mm grid:
  // the closing return move brings the head back to the start point
  // with net zero displacement. Zero-length deltas are not emitted.
  var b = stats.bounds;
  var lines = [
    "; LaserForge " + VERSION + " -- SVG -> GRBL gcode",
    "; Source: " + meta.source,
    "; Mode: " + meta.mode,
    "; Start mode: current position",
    "; Anchor: " + meta.anchor_x + " / " + meta.anchor_y +
      " (those design edges sit on the head)",
    "; Bounds (relative to start): X" + fmt(b[0]) + " Y" + fmt(b[1]) +
      " to X" + fmt(b[2]) + " Y" + fmt(b[3]),
    "; Burn distance: " + pyFixed(stats.burn, 1) + " mm",
    "; Travel distance: " + pyFixed(stats.travel, 1) + " mm",
    "; Estimated time: " + pyFixed(stats.minutes, 2) + " min",
    "; Layers: " + meta.layer_names.join(", "),
    "G21",
    "G91",
    "M4",
    "S0"
  ];
  lines.splice.apply(lines, [3, 0].concat(rotaryHeaderLines(meta)));
  var pos = [0.0, 0.0];

  function move(code, target, extra) {
    var t = [r3(target[0]), r3(target[1])];
    var d = [t[0] - pos[0], t[1] - pos[1]];
    if (d[0] === 0.0 && d[1] === 0.0) return false;
    lines.push(code + " X" + fmt(d[0]) + " Y" + fmt(d[1]) + (extra || ""));
    pos = t;
    return true;
  }

  var group = null;
  strokes.forEach(function (st) {
    var key = st.layer + "\x00" + st.op + "\x00" + st.passNo;
    if (key !== group) {
      group = key;
      lines.push(layerComment(st));
    }
    move("G0", st.points[0], "");
    var first = true;
    for (var i = 1; i < st.points.length; i++) {
      var extra = first ? " F" + fmtFeed(st.feed) + " S" + st.s : "";
      if (move("G1", st.points[i], extra)) first = false;
    }
    lines.push("S0");
  });
  lines.push("; End of job", "S0", "M5");
  move("G0", [0.0, 0.0], "");
  return lines.join("\n") + "\n";
}

function verifyGcodeCurrent(text, bedW, bedH) {
  // Re-read current-position output: integrate the G91 deltas from the
  // frame origin, recompute the burn bounds, check the design's span
  // fits within the bed size as a maximum span, and confirm the head
  // ends back at the start point.
  var x = 0.0, y = 0.0, s = 0;
  var bounds = null;
  var problems = [];
  var sawG91 = false;

  function grow(p) {
    if (bounds === null) {
      bounds = [p[0], p[1], p[0], p[1]];
    } else {
      bounds[0] = Math.min(bounds[0], p[0]);
      bounds[1] = Math.min(bounds[1], p[1]);
      bounds[2] = Math.max(bounds[2], p[0]);
      bounds[3] = Math.max(bounds[3], p[1]);
    }
  }

  text.split("\n").forEach(function (raw) {
    var line = raw.trim();
    if (!line || line.charAt(0) === ";") return;
    var cmd = null;
    var dx = 0.0, dy = 0.0, ns = s;
    line.split(/\s+/).forEach(function (tok) {
      var head = tok.charAt(0);
      if (head === "G") {
        cmd = tok;
        if (tok === "G91") sawG91 = true;
        else if (tok === "G90") {
          problems.push("G90 found in current-position output");
        }
      } else if (head === "X") dx = parseFloat(tok.slice(1));
      else if (head === "Y") dy = parseFloat(tok.slice(1));
      else if (head === "S") ns = Math.trunc(parseFloat(tok.slice(1)));
    });
    if (cmd === "G0" || cmd === "G1") {
      var ox = x, oy = y;
      x = x + dx;
      y = y + dy;
      if (cmd === "G1" && ns > 0) {
        grow([ox, oy]);
        grow([x, y]);
      }
    }
    s = ns;
  });
  if (!sawG91) problems.push("current-position output has no G91");
  if (bounds !== null) {
    var w = bounds[2] - bounds[0];
    var h = bounds[3] - bounds[1];
    if (w > bedW + 0.001 || h > bedH + 0.001) {
      problems.push("design span " + fmt(w) + " x " + fmt(h) +
                    " mm exceeds the bed size (" + pyFixed(bedW, 0) +
                    " x " + pyFixed(bedH, 0) + " mm) as a maximum span");
    }
  }
  if (Math.abs(x) > 0.0015 || Math.abs(y) > 0.0015) {
    problems.push("head does not return to the start point " +
                  "(net displacement X" + fmt(x) + " Y" + fmt(y) + ")");
  }
  return { bounds: bounds, problems: problems };
}

// ---------------------------------------------------------------------------
// Machine profiles: mm_per_rotation is a machine+chuck constant. It
// lives in a JSON profile store -- never in a job file or on a cup --
// because it does not change when the cup does. (In this port the
// store is a plain object; the CLI persists it to disk exactly like
// the Python store, the web app persists it to localStorage.)
// ---------------------------------------------------------------------------

var SEED_PROFILES = {
  "falcon-pro": {
    name: "Falcon Pro",
    machine: "CR-Laser Falcon Pro 10W (GRBL, chuck rotary)",
    mm_per_rotation: 44.5,
    status: "measured 2026-11-21; confirmation proof burn pending",
    notes: "Measured via the LightBurn route on 2026-11-21: the " +
      "Williams logo's intended wrap was 2.5 in and it burned " +
      "4.125 in at mm_per_rotation 73.4 (a derived value); " +
      "true = 73.4 x 2.5 / 4.125 = 44.48, set to 44.5. One " +
      "constant for the machine+chuck; per cup, only the " +
      "circumference changes."
  }
};

function seedStore() {
  var profiles = {};
  Object.keys(SEED_PROFILES).forEach(function (k) {
    var src = SEED_PROFILES[k], copy = {};
    Object.keys(src).forEach(function (kk) { copy[kk] = src[kk]; });
    profiles[k] = copy;
  });
  return { profiles: profiles, calibration_pending: null };
}

function checkStore(store, pathLabel) {
  if (!store || typeof store !== "object" ||
      !store.profiles || typeof store.profiles !== "object") {
    die("profiles file " + pathLabel +
        ": expected an object with a 'profiles' object");
  }
  if (!("calibration_pending" in store)) store.calibration_pending = null;
  return store;
}

function getProfile(store, key, pathLabel) {
  var prof = store.profiles[key];
  if (prof === undefined || prof === null) {
    var have = Object.keys(store.profiles).sort().join(", ") || "none";
    die("profile " + pyRepr(key) + " not found in " + pathLabel +
        " (have: " + have + ")");
  }
  if (prof.mm_per_rotation === null || prof.mm_per_rotation === undefined) {
    die("profile " + pyRepr(key) + " in " + pathLabel +
        " has no mm_per_rotation -- calibrate it first " +
        "(--calibrate-generate / --calibrate-apply)");
  }
  return prof;
}

function resolveCircumference(cands) {
  // cands: list of [label, mm value]. The circumference may be given
  // exactly one way.
  if (cands.length > 1) {
    die("give the circumference only one way (got " +
        cands.map(function (c) { return c[0]; }).join(" and ") + ")");
  }
  return cands.length ? cands[0][1] : null;
}

function circumferenceCandidates(mm, inch, diameterIn) {
  var cands = [];
  if (mm !== null && mm !== undefined) {
    cands.push(["--circumference-mm", Number(mm)]);
  }
  if (inch !== null && inch !== undefined) {
    cands.push(["--circumference-in", Number(inch) * IN_MM]);
  }
  if (diameterIn !== null && diameterIn !== undefined) {
    cands.push(["--diameter-in", Number(diameterIn) * IN_MM * Math.PI]);
  }
  return cands;
}

// ---------------------------------------------------------------------------
// Calibration core (shared by the CLI and the web app)
// ---------------------------------------------------------------------------

function calibrationGcode(opts) {
  // Build the calibration burn file: one straight line whose intended
  // wrap arc is the test length, emitted with the profile's CURRENT
  // mm_per_rotation, current-position start. opts: {profileKey,
  // profile, circumferenceMm, testArcMm, speed, power}. Returns
  // {gcode, factor, emitted}. Throws LFError (code 3) if the file
  // fails self-verification, like the Python engine.
  var prof = opts.profile;
  var circ = opts.circumferenceMm;
  var arcMm = opts.testArcMm;
  var mpr = Number(prof.mm_per_rotation);
  var factor = mpr / circ;
  var stroke = new Stroke([[0.0, 0.0], [0.0, arcMm * factor]],
                          Number(opts.speed),
                          pyRoundHalfEven(Number(opts.power) * 10.0, 0),
                          Number(opts.power), "calibration", "line", 1, 1);
  var strokes = [stroke];
  var stats = analyze(strokes);
  var meta = {
    source: "calibration line (generated; no input SVG)",
    mode: "rotary",
    anchor_x: "left",
    anchor_y: "bottom",
    layer_names: ["calibration"],
    rotary: {
      profile_key: opts.profileKey,
      profile_name: prof.name !== undefined ? prof.name : opts.profileKey,
      mm_per_rotation: mpr,
      status: prof.status || "",
      circumference_mm: circ,
      factor: factor
    },
    calibration: { test_arc_mm: arcMm }
  };
  var text = renderGcodeCurrent(strokes, stats, meta);
  var ver = verifyGcodeCurrent(text, DEFAULT_BED[0], DEFAULT_BED[1]);
  var problems = ver.problems.slice();
  if (ver.bounds !== null) {
    var emitted = ver.bounds[3] - ver.bounds[1];
    if (Math.abs(emitted - arcMm * factor) > 0.0025) {
      problems.push("emitted wrap travel " + fmt(emitted) +
                    " mm does not match intended arc x wrap scale (" +
                    fmt(arcMm * factor) + " mm)");
    }
  }
  if (problems.length) {
    var err = new LFError("verification failed -- output NOT written (" +
                          problems.length + " problem(s))", 3);
    err.verifyProblems = problems;
    throw err;
  }
  return { gcode: text, factor: factor, emitted: arcMm * factor };
}

function calibrationApplyMath(used, intended, measured) {
  // true = used x intended / measured; stored rounded to 2 decimals.
  var exact = used * intended / measured;
  return { exact: exact, value: pyRoundHalfEven(exact, 2) };
}

function todayISO() {
  var d = new Date();
  function p(v) { return String(v).padStart(2, "0"); }
  return d.getFullYear() + "-" + p(d.getMonth() + 1) + "-" + p(d.getDate());
}

function nowISOSeconds() {
  var d = new Date();
  function p(v) { return String(v).padStart(2, "0"); }
  return todayISO() + "T" + p(d.getHours()) + ":" + p(d.getMinutes()) +
         ":" + p(d.getSeconds());
}

// ---------------------------------------------------------------------------
// Job assembly + the shared run core (mirrors laserforge.py main()).
// runJob(opts, log) with opts {svgText, sourceName, job, store,
// storeLabel} returns {gcode, stats, bounds, meta, statsLines} and
// throws LFError on any failure. `log(line)` receives the report lines
// in the exact order the Python engine prints them.
// ---------------------------------------------------------------------------

var ANCHOR_PAIRS = {
  center: ["center", "center"],
  min: ["left", "bottom"],
  max: ["right", "top"]
};

function anchorPair(name) {
  var pair = ANCHOR_PAIRS[name];
  if (!pair) die("anchor must be 'center', 'min' or 'max' (got " +
                 pyRepr(name) + ")");
  return pair;
}

function normalizeJob(raw, jobLabel, deferValidate) {
  // The job-file surface: a raw parsed JSON job object -> the internal
  // job shape (defaults mirror load_job()). The CLI defers validation
  // until after its overrides are applied, like load_job() does.
  var job = {
    mode: "flat", offset: [0.0, 0.0], bed: DEFAULT_BED.slice(),
    start_mode: "current", anchor_x: "left", anchor_y: "bottom",
    layers: [], profile: "falcon-pro", circumference_mm: null
  };
  if (raw.mode !== undefined) job.mode = raw.mode;
  if (raw.offset !== undefined) {
    job.offset = [Number(raw.offset.x || 0.0), Number(raw.offset.y || 0.0)];
  }
  if (raw.bed !== undefined) {
    job.bed = [Number(raw.bed.width !== undefined ? raw.bed.width
                        : DEFAULT_BED[0]),
               Number(raw.bed.height !== undefined ? raw.bed.height
                        : DEFAULT_BED[1])];
  }
  var startMode = raw.start_mode !== undefined ? raw.start_mode
                                                 : raw.start_from;
  if (startMode !== undefined && startMode !== null) {
    job.start_mode = startMode;
  }
  if (raw.anchor !== undefined) {
    var pair = anchorPair(raw.anchor);
    job.anchor_x = pair[0];
    job.anchor_y = pair[1];
  }
  if (raw.anchor_x !== undefined) job.anchor_x = raw.anchor_x;
  if (raw.anchor_y !== undefined) job.anchor_y = raw.anchor_y;
  if (raw.profile !== undefined) job.profile = raw.profile;
  var jobCands = circumferenceCandidates(
    raw.circumference_mm !== undefined ? raw.circumference_mm : null,
    raw.circumference_in !== undefined ? raw.circumference_in : null,
    raw.diameter_in !== undefined ? raw.diameter_in : null);
  if (jobCands.length) job.circumference_mm = resolveCircumference(jobCands);
  if (!raw.layers || !raw.layers.length) {
    die("job file " + (jobLabel || "job") + ": no layers defined");
  }
  job.layers = raw.layers.map(function (l, i) {
    return normalizeLayer(l, i);
  });
  if (!deferValidate) validateJob(job);
  return job;
}

function validateJob(job) {
  if (job.start_mode !== "current" && job.start_mode !== "absolute") {
    die("start_mode must be 'current' or 'absolute' (got " +
        pyRepr(job.start_mode) + ")");
  }
  if (["left", "center", "right"].indexOf(job.anchor_x) === -1) {
    die("anchor_x must be 'left', 'center' or 'right' (got " +
        pyRepr(job.anchor_x) + ")");
  }
  if (["top", "center", "bottom"].indexOf(job.anchor_y) === -1) {
    die("anchor_y must be 'top', 'center' or 'bottom' (got " +
        pyRepr(job.anchor_y) + ")");
  }
}

function runJob(opts, log) {
  log = log || function () {};
  var job = opts.job;

  if (job.mode !== "flat" && job.mode !== "rotary") {
    die("mode must be 'flat' or 'rotary' (got " + pyRepr(job.mode) + ")");
  }
  var rotary = null;
  if (job.mode === "rotary") {
    if (job.circumference_mm === null ||
        job.circumference_mm === undefined) {
      die("mode 'rotary' needs the cup circumference: " +
          "--circumference-mm, --circumference-in or --diameter-in " +
          "(or the matching job-file keys)");
    }
    if (job.circumference_mm <= 0) die("circumference must be > 0");
    var storeLabel = opts.storeLabel || "profiles.json";
    // The store is loaded lazily HERE, at profile resolution — the
    // Python engine seeds/creates the profiles file only at this
    // point, so flat runs and early rotary refusals leave no file.
    var store = opts.store;
    if (!store && opts.loadStore) store = opts.loadStore();
    var prof = getProfile(store, job.profile, storeLabel);
    var mpr = Number(prof.mm_per_rotation);
    rotary = {
      profile_key: job.profile,
      profile_name: prof.name !== undefined ? prof.name : job.profile,
      mm_per_rotation: mpr,
      status: prof.status || "",
      circumference_mm: job.circumference_mm,
      factor: mpr / job.circumference_mm
    };
  }

  var loaded = loadSVGText(opts.svgText, opts.sourceName, opts.inputLabel);
  var shapes = loaded.shapes;
  var report = loaded.report;
  if (!shapes.length) {
    die((opts.inputLabel || opts.sourceName) + ": no usable geometry found");
  }
  shapes = cleanupShapes(shapes, report);
  report.forEach(log);

  var ox = job.offset[0], oy = job.offset[1];
  if (ox || oy) {
    shapes.forEach(function (sh) {
      sh.points = sh.points.map(function (p) { return [p[0] + ox, p[1] + oy]; });
    });
  }

  var layerReport = [];
  var strokes = buildStrokes(job, shapes, layerReport);
  layerReport.forEach(log);
  if (!strokes.length) die("nothing to burn: the job produced no toolpaths");

  if (job.start_mode === "current") {
    // Translate the whole job so the anchor point is the frame origin.
    var db = strokesBounds(strokes);
    var ref = anchorReference(db, job.anchor_x, job.anchor_y);
    strokes.forEach(function (st) {
      st.points = st.points.map(function (p) {
        return [p[0] - ref[0], p[1] - ref[1]];
      });
    });
  }

  var designYSpan = null;
  if (rotary !== null) {
    // Rotary: the design's Y is the wrap (circumference) direction.
    // Scale it by mm_per_rotation / circumference; X is unscaled. The
    // anchor was resolved in design terms above, before this scaling.
    var sb = strokesBounds(strokes);
    designYSpan = sb[3] - sb[1];
    var f = rotary.factor;
    strokes.forEach(function (st) {
      st.points = st.points.map(function (p) { return [p[0], p[1] * f]; });
    });
  }

  var stats = analyze(strokes);
  var meta = {
    source: opts.sourceName,
    mode: job.mode,
    start_mode: job.start_mode,
    anchor_x: job.anchor_x,
    anchor_y: job.anchor_y,
    layer_names: job.layers.filter(function (l) { return l.output; })
      .map(function (l) { return l.name; }),
    rotary: rotary
  };
  var text = job.start_mode === "current"
    ? renderGcodeCurrent(strokes, stats, meta)
    : renderGcode(strokes, stats, meta);

  var bedW = job.bed[0], bedH = job.bed[1];
  var ver = job.start_mode === "current"
    ? verifyGcodeCurrent(text, bedW, bedH)
    : verifyGcode(text, bedW, bedH);
  var bounds = ver.bounds;
  var problems = ver.problems.slice();
  if (rotary !== null && bounds !== null) {
    var expectedWrap = stats.bounds[3] - stats.bounds[1];
    var emittedWrap = bounds[3] - bounds[1];
    if (Math.abs(emittedWrap - expectedWrap) > 0.0025) {
      problems.push("rotary wrap travel mismatch: emitted Y extent " +
        fmt(emittedWrap) + " mm, expected " + fmt(expectedWrap) +
        " mm (design wrap span " + fmt(designYSpan) +
        " mm x mm_per_rotation " + fmt(rotary.mm_per_rotation) +
        " / circumference " + fmt(rotary.circumference_mm) + ")");
    }
  }
  if (problems.length) {
    var err = new LFError("verification failed -- output NOT written (" +
                          problems.length + " problem(s))", 3);
    err.verifyProblems = problems;
    throw err;
  }
  if (bounds === null) {
    die("verification failed -- no burn moves found in generated output", 3);
  }

  // The console summary the Python engine prints after writing.
  var statsLines = [];
  if (job.start_mode === "current") {
    statsLines.push("bounds (relative to start point): X " + fmt(bounds[0]) +
                    ".." + fmt(bounds[2]) + "  Y " + fmt(bounds[1]) + ".." +
                    fmt(bounds[3]));
  } else {
    statsLines.push("bounds (design): X " + fmt(bounds[0]) + ".." +
                    fmt(bounds[2]) + "  Y " + fmt(bounds[1]) + ".." +
                    fmt(bounds[3]));
  }
  statsLines.push("burn distance: " + pyFixed(stats.burn, 1) + " mm");
  statsLines.push("travel distance: " + pyFixed(stats.travel, 1) + " mm");
  statsLines.push("estimated time: " + pyFixed(stats.minutes, 2) +
                  " min (" + pyFixed(stats.minutes * 60.0, 1) + " s)  " +
                  "[estimate: sum(length/feed); rapids at " +
                  Math.trunc(RAPID_FEED) + " mm/min]");
  if (job.start_mode === "current") {
    statsLines.push("emitted bounds: X " + fmt(bounds[0]) + ".." +
      fmt(bounds[2]) + "  Y " + fmt(bounds[1]) + ".." + fmt(bounds[3]) +
      " (span " + fmt(bounds[2] - bounds[0]) + " x " +
      fmt(bounds[3] - bounds[1]) + " mm) -- span within bed max " +
      pyG(bedW) + " x " + pyG(bedH) + " mm; head returns to the start point");
  } else {
    statsLines.push("emitted bounds: X " + fmt(bounds[0]) + ".." +
      fmt(bounds[2]) + "  Y " + fmt(bounds[1]) + ".." + fmt(bounds[3]) +
      " (" + fmt(bounds[2] - bounds[0]) + " x " +
      fmt(bounds[3] - bounds[1]) + " mm) -- inside bed " + pyG(bedW) +
      " x " + pyG(bedH) + " mm");
  }
  if (rotary !== null) {
    statsLines.push("rotary: profile '" + rotary.profile_key +
      "', mm per rotation " + fmt(rotary.mm_per_rotation) +
      " mm, circumference " + fmt(rotary.circumference_mm) + " mm (" +
      fmt(rotary.circumference_mm / IN_MM) + " in), wrap scale Y x " +
      pyFixed(rotary.factor, 6));
    statsLines.push("emitted wrap travel: Y extent " +
      fmt(bounds[3] - bounds[1]) + " mm (design wrap span " +
      fmt(designYSpan) + " mm x " + pyFixed(rotary.factor, 6) + ")");
  }

  return {
    gcode: text,
    stats: stats,
    bounds: bounds,
    meta: meta,
    strokes: strokes,
    shapes: shapes,
    statsLines: statsLines
  };
}

// ---------------------------------------------------------------------------
// Node CLI -- mirrors laserforge.py's argparse surface and main().
// Only invoked when this file is run directly under Node; the browser
// app uses the engine functions above and never touches this section.
// ---------------------------------------------------------------------------

function pyJsonStringify(value) {
  // json.dump(..., indent=2) with Python float formatting (44.5 ->
  // "44.5", 20.0 -> "20.0") and ensure_ascii string escaping.
  function escStr(s) {
    var out = '"';
    for (var i = 0; i < s.length; i++) {
      var code = s.charCodeAt(i);
      var ch = s[i];
      if (ch === '"') out += '\\"';
      else if (ch === "\\") out += "\\\\";
      else if (ch === "\n") out += "\\n";
      else if (ch === "\r") out += "\\r";
      else if (ch === "\t") out += "\\t";
      else if (ch === "\b") out += "\\b";
      else if (ch === "\f") out += "\\f";
      else if (code < 0x20 || code > 0x7e) {
        out += "\\u" + code.toString(16).padStart(4, "0");
      } else out += ch;
    }
    return out + '"';
  }
  function ser(v, level) {
    var pad = "  ".repeat(level);
    var padIn = "  ".repeat(level + 1);
    if (v === null) return "null";
    if (typeof v === "boolean") return v ? "true" : "false";
    if (typeof v === "number") return pyStrFloat(v);
    if (typeof v === "string") return escStr(v);
    if (Array.isArray(v)) {
      if (!v.length) return "[]";
      return "[\n" + v.map(function (item) {
        return padIn + ser(item, level + 1);
      }).join(",\n") + "\n" + pad + "]";
    }
    var keys = Object.keys(v);
    if (!keys.length) return "{}";
    return "{\n" + keys.map(function (k) {
      return padIn + escStr(k) + ": " + ser(v[k], level + 1);
    }).join(",\n") + "\n" + pad + "}";
  }
  return ser(value, 0);
}

function cliMain(argv) {
  var fs = require("fs");
  var pathMod = require("path");
  var DEFAULT_PROFILES_PATH = pathMod.join(__dirname, "profiles.json");

  function loadProfilesFile(p) {
    if (!fs.existsSync(p)) {
      var store = seedStore();
      saveProfilesFile(p, store);
      return store;
    }
    var raw;
    try {
      raw = JSON.parse(fs.readFileSync(p, "utf8"));
    } catch (exc) {
      die("cannot read profiles file " + p + ": " + exc.message);
    }
    return checkStore(raw, p);
  }

  function saveProfilesFile(p, store) {
    try {
      fs.writeFileSync(p, pyJsonStringify(store) + "\n");
    } catch (exc) {
      die("cannot write profiles file " + p + ": " + exc.message);
    }
  }

  function showProfile(args) {
    var p = args.profiles || DEFAULT_PROFILES_PATH;
    var key = args.profile || "falcon-pro";
    var store = loadProfilesFile(p);
    var prof = getProfile(store, key, p);
    console.log("profile '" + key + "' (" + p + ")");
    console.log("  name: " + (prof.name !== undefined ? prof.name : key));
    if (prof.machine) console.log("  machine: " + prof.machine);
    console.log("  mm_per_rotation: " + fmt(Number(prof.mm_per_rotation)) +
                " mm");
    console.log("  status: " + (prof.status || ""));
    if (prof.notes) console.log("  notes: " + prof.notes);
    var pend = store.calibration_pending;
    if (pend) {
      console.log("  calibration pending: test arc " +
        fmt(pend.test_arc_mm) + " mm at mm_per_rotation " +
        fmt(pend.mm_per_rotation_used) + " on circumference " +
        fmt(pend.circumference_mm) +
        " mm -- measure the burned arc and run --calibrate-apply");
    }
    return 0;
  }

  function calibrateGenerate(args) {
    var p = args.profiles || DEFAULT_PROFILES_PATH;
    var key = args.profile || "falcon-pro";
    var store = loadProfilesFile(p);
    var prof = getProfile(store, key, p);
    var circ = resolveCircumference(circumferenceCandidates(
      args.circumference_mm, args.circumference_in, args.diameter_in));
    if (circ === null) {
      die("--calibrate-generate needs the cup circumference: " +
          "--circumference-mm, --circumference-in or --diameter-in");
    }
    if (circ <= 0) die("circumference must be > 0");
    if (args.test_arc_mm !== null && args.test_arc_in !== null) {
      die("give the test arc only one way (--test-arc-in or " +
          "--test-arc-mm)");
    }
    var arcMm;
    if (args.test_arc_mm !== null) arcMm = Number(args.test_arc_mm);
    else if (args.test_arc_in !== null) {
      arcMm = Number(args.test_arc_in) * IN_MM;
    } else arcMm = 2.5 * IN_MM;
    if (arcMm <= 0) die("test arc must be > 0");
    var mpr = Number(prof.mm_per_rotation);
    var res = calibrationGcode({
      profileKey: key, profile: prof, circumferenceMm: circ,
      testArcMm: arcMm, speed: args.speed, power: args.power
    });
    var outPath = args.output || "calibration.gcode";
    fs.writeFileSync(outPath, res.gcode);
    store.calibration_pending = {
      profile: key,
      circumference_mm: circ,
      test_arc_mm: arcMm,
      mm_per_rotation_used: mpr,
      generated: nowISOSeconds(),
      output: outPath
    };
    saveProfilesFile(p, store);
    console.log("calibration file for profile '" + key +
                "' (mm_per_rotation " + fmt(mpr) + " mm)");
    console.log("circumference: " + fmt(circ) + " mm (" +
                fmt(circ / IN_MM) + " in); intended test arc: " +
                fmt(arcMm) + " mm (" + fmt(arcMm / IN_MM) + " in)");
    console.log("emitted wrap travel: " + fmt(arcMm * res.factor) +
                " mm (arc x mm_per_rotation / circumference)");
    console.log("wrote: " + outPath);
    console.log("next: burn it on tape, measure the burned arc, then " +
                "run --calibrate-apply --measured-in <your measurement>");
    return 0;
  }

  function calibrateApply(args) {
    var p = args.profiles || DEFAULT_PROFILES_PATH;
    var store = loadProfilesFile(p);
    var pend = store.calibration_pending;
    if (!pend) {
      die("no calibration is pending -- run --calibrate-generate " +
          "first, burn the line on tape, measure the arc, then apply");
    }
    if (args.measured_mm !== null && args.measured_in !== null) {
      die("give the measurement only one way (--measured-in or " +
          "--measured-mm)");
    }
    var measured;
    if (args.measured_in !== null) {
      measured = Number(args.measured_in) * IN_MM;
    } else if (args.measured_mm !== null) {
      measured = Number(args.measured_mm);
    } else {
      die("--calibrate-apply needs the measured arc: --measured-in or " +
          "--measured-mm");
    }
    if (measured <= 0) die("measured arc must be > 0");
    var key = pend.profile;
    var prof = getProfile(store, key, p);
    var used = Number(pend.mm_per_rotation_used);
    var intended = Number(pend.test_arc_mm);
    var math = calibrationApplyMath(used, intended, measured);
    var oldValue = Number(prof.mm_per_rotation);
    var today = todayISO();
    prof.mm_per_rotation = math.value;
    prof.status = "measured " + today;
    var note = "Calibrated " + today + ": test arc " + fmt(intended) +
      " mm intended, measured " + fmt(measured) +
      " mm at mm_per_rotation " + fmt(used) + " -> true = " + fmt(used) +
      " x " + fmt(intended) + " / " + fmt(measured) + " = " +
      pyFixed(math.exact, 4) + ", stored " + fmt(math.value) + ".";
    prof.notes = ((prof.notes || "") + " " + note).trim();
    store.calibration_pending = null;
    saveProfilesFile(p, store);
    console.log("calibration applied to profile '" + key + "':");
    console.log("  mm_per_rotation = used x intended / measured");
    console.log("                  = " + fmt(used) + " x " + fmt(intended) +
                " / " + fmt(measured));
    console.log("                  = " + pyFixed(math.exact, 4) + " -> " +
                fmt(math.value) + " mm (was " + fmt(oldValue) + " mm)");
    console.log("  status: " + prof.status);
    return 0;
  }

  function parseBed(text) {
    var m = /^\s*([\d.]+)\s*x\s*([\d.]+)\s*$/.exec(text || "");
    if (!m) die("--bed must look like 400x415 (got " + pyRepr(text) + ")");
    return [parseFloat(m[1]), parseFloat(m[2])];
  }

  function loadJobCli(args) {
    var job = {
      mode: "flat", offset: [0.0, 0.0], bed: DEFAULT_BED.slice(),
      start_mode: "current", anchor_x: "left", anchor_y: "bottom",
      layers: [], profile: "falcon-pro", circumference_mm: null
    };
    if (args.job) {
      var raw;
      try {
        raw = JSON.parse(fs.readFileSync(args.job, "utf8"));
      } catch (exc) {
        die("cannot read job file " + args.job + ": " + exc.message);
      }
      job = normalizeJob(raw, args.job, true);
    } else {
      var sel = null;
      if (args.select) {
        sel = [];
        args.select.split(",").forEach(function (tok) {
          tok = tok.trim();
          if (!tok) return;
          sel.push(/^\d+$/.test(tok) ? parseInt(tok, 10) : tok);
        });
      }
      var layer = {
        name: "layer-1", select: sel,
        ops: [{ op: args.op, speed: args.speed, power: args.power,
                passes: args.passes, spacing: args.spacing,
                angle: args.angle, outline: args.outline }]
      };
      job.layers = [normalizeLayer(layer, 0)];
    }
    // CLI overrides
    if (args.mode) job.mode = args.mode;
    if (args.offset_x !== null || args.offset_y !== null) {
      job.offset = [
        args.offset_x !== null ? args.offset_x : job.offset[0],
        args.offset_y !== null ? args.offset_y : job.offset[1]
      ];
    }
    if (args.bed) job.bed = parseBed(args.bed);
    if (args.start_mode) job.start_mode = args.start_mode;
    if (args.anchor) {
      var pair = anchorPair(args.anchor);
      job.anchor_x = pair[0];
      job.anchor_y = pair[1];
    }
    if (args.anchor_x) job.anchor_x = args.anchor_x;
    if (args.anchor_y) job.anchor_y = args.anchor_y;
    if (args.profile) job.profile = args.profile;
    var cliCands = circumferenceCandidates(
      args.circumference_mm, args.circumference_in, args.diameter_in);
    if (cliCands.length) {
      job.circumference_mm = resolveCircumference(cliCands);
    }
    validateJob(job);
    return job;
  }

  function splitext(p) {
    var slash = Math.max(p.lastIndexOf("/"), p.lastIndexOf("\\"));
    var base = p.slice(slash + 1);
    var dot = base.lastIndexOf(".");
    if (dot === -1) return [p, ""];
    var before = base.slice(0, dot);
    if (before === "" || /^\.+$/.test(before)) return [p, ""];
    return [p.slice(0, slash + 1 + dot), base.slice(dot)];
  }

  // ---- argument parsing (argparse-shaped; error TEXT for malformed
  // command lines is argparse's own and is not part of the port's
  // parity contract -- exit codes are: 2 for bad input, as Python) --
  var VALUE_FLAGS = {
    "-o": "output", "--output": "output", "--job": "job",
    "--mode": "mode", "--profile": "profile", "--profiles": "profiles",
    "--circumference-mm": "circumference_mm",
    "--circumference-in": "circumference_in",
    "--diameter-in": "diameter_in",
    "--test-arc-in": "test_arc_in", "--test-arc-mm": "test_arc_mm",
    "--measured-in": "measured_in", "--measured-mm": "measured_mm",
    "--op": "op", "--speed": "speed", "--power": "power",
    "--passes": "passes", "--spacing": "spacing", "--angle": "angle",
    "--select": "select", "--start-mode": "start_mode",
    "--anchor": "anchor", "--anchor-x": "anchor_x",
    "--anchor-y": "anchor_y", "--offset-x": "offset_x",
    "--offset-y": "offset_y", "--bed": "bed"
  };
  var BOOL_FLAGS = {
    "--show-profile": "show_profile",
    "--calibrate-generate": "calibrate_generate",
    "--calibrate-apply": "calibrate_apply",
    "--outline": "outline"
  };
  var FLOAT_KEYS = { speed: 1, power: 1, spacing: 1, angle: 1,
                     circumference_mm: 1, circumference_in: 1,
                     diameter_in: 1, test_arc_in: 1, test_arc_mm: 1,
                     measured_in: 1, measured_mm: 1, offset_x: 1,
                     offset_y: 1 };
  var args = {
    input: null, output: null, job: null, mode: null, profile: null,
    profiles: null, show_profile: false, calibrate_generate: false,
    calibrate_apply: false, circumference_mm: null,
    circumference_in: null, diameter_in: null, test_arc_in: null,
    test_arc_mm: null, measured_in: null, measured_mm: null,
    op: "line", speed: 1000.0, power: 50.0, passes: 1, spacing: 0.1,
    angle: 0.0, outline: false, select: null, start_mode: null,
    anchor: null, anchor_x: null, anchor_y: null, offset_x: null,
    offset_y: null, bed: null
  };
  for (var ai = 0; ai < argv.length; ai++) {
    var a = argv[ai];
    var eq = a.indexOf("=");
    var flag = eq !== -1 && a.charAt(0) === "-" ? a.slice(0, eq) : a;
    var inlineVal = eq !== -1 && a.charAt(0) === "-" ? a.slice(eq + 1)
                                                     : null;
    if (BOOL_FLAGS[flag] !== undefined && inlineVal === null) {
      args[BOOL_FLAGS[flag]] = true;
    } else if (VALUE_FLAGS[flag] !== undefined) {
      var key = VALUE_FLAGS[flag];
      var val = inlineVal;
      if (val === null) {
        ai++;
        if (ai >= argv.length) {
          die("argument " + flag + ": expected one argument", 2);
        }
        val = argv[ai];
      }
      if (FLOAT_KEYS[key]) {
        var fv = Number(val);
        if (Number.isNaN(fv)) {
          die("argument " + flag + ": invalid float value: " +
              pyRepr(val), 2);
        }
        args[key] = fv;
      } else if (key === "passes") {
        var pv = Number(val);
        if (!Number.isInteger(pv)) {
          die("argument --passes: invalid int value: " + pyRepr(val), 2);
        }
        args.passes = pv;
      } else {
        args[key] = val;
      }
    } else if (a.charAt(0) === "-") {
      die("unrecognized arguments: " + a, 2);
    } else if (args.input === null) {
      args.input = a;
    } else {
      die("unrecognized arguments: " + a, 2);
    }
  }
  // Choice validation (argparse `choices=`).
  if (["line", "fill", "cut"].indexOf(args.op) === -1) {
    die("argument --op: invalid choice: " + pyRepr(args.op), 2);
  }
  if (args.start_mode !== null &&
      ["current", "absolute"].indexOf(args.start_mode) === -1) {
    die("argument --start-mode: invalid choice: " +
        pyRepr(args.start_mode), 2);
  }
  if (args.anchor !== null &&
      ["center", "min", "max"].indexOf(args.anchor) === -1) {
    die("argument --anchor: invalid choice: " + pyRepr(args.anchor), 2);
  }
  if (args.anchor_x !== null &&
      ["left", "center", "right"].indexOf(args.anchor_x) === -1) {
    die("argument --anchor-x: invalid choice: " + pyRepr(args.anchor_x), 2);
  }
  if (args.anchor_y !== null &&
      ["top", "center", "bottom"].indexOf(args.anchor_y) === -1) {
    die("argument --anchor-y: invalid choice: " +
        pyRepr(args.anchor_y), 2);
  }

  try {
    // Profile / calibration steps need no input SVG and no job.
    if (args.show_profile) return showProfile(args);
    if (args.calibrate_apply) return calibrateApply(args);
    if (args.calibrate_generate) return calibrateGenerate(args);
    if (!args.input) {
      die("an input SVG is required (or use --show-profile / " +
          "--calibrate-generate / --calibrate-apply)");
    }

    var job = loadJobCli(args);
    var storeLabel = args.profiles || DEFAULT_PROFILES_PATH;
    var svgText = fs.readFileSync(args.input, "utf8");
    var baseName = pathMod.basename(args.input);
    var result = runJob({
      svgText: svgText, sourceName: baseName, inputLabel: args.input,
      job: job, store: null,
      loadStore: function () { return loadProfilesFile(storeLabel); },
      storeLabel: storeLabel
    }, function (line) { console.log(line); });

    var outPath = args.output || splitext(args.input)[0] + ".gcode";
    fs.writeFileSync(outPath, result.gcode);
    result.statsLines.forEach(function (line) { console.log(line); });
    console.log("wrote: " + outPath);
    return 0;
  } catch (e) {
    if (e instanceof LFError) {
      if (e.verifyProblems) {
        e.verifyProblems.slice(0, 10).forEach(function (p) {
          process.stderr.write("verify: " + p + "\n");
        });
      }
      process.stderr.write("laserforge: error: " + e.message + "\n");
      return e.code;
    }
    process.stderr.write(String((e && e.stack) || e) + "\n");
    return 1;
  }
}

// ---------------------------------------------------------------------------
// Exports
// ---------------------------------------------------------------------------

var LaserForge = {
  VERSION: VERSION,
  DEFAULT_BED: DEFAULT_BED,
  IN_MM: IN_MM,
  LFError: LFError,
  fmt: fmt,
  fmtFeed: fmtFeed,
  parseXML: parseXML,
  loadSVGText: loadSVGText,
  cleanupShapes: cleanupShapes,
  hatchSegments: hatchSegments,
  normalizeOp: normalizeOp,
  normalizeLayer: normalizeLayer,
  normalizeJob: normalizeJob,
  buildStrokes: buildStrokes,
  analyze: analyze,
  renderGcode: renderGcode,
  renderGcodeCurrent: renderGcodeCurrent,
  verifyGcode: verifyGcode,
  verifyGcodeCurrent: verifyGcodeCurrent,
  strokesBounds: strokesBounds,
  anchorReference: anchorReference,
  seedStore: seedStore,
  checkStore: checkStore,
  getProfile: getProfile,
  resolveCircumference: resolveCircumference,
  circumferenceCandidates: circumferenceCandidates,
  calibrationGcode: calibrationGcode,
  calibrationApplyMath: calibrationApplyMath,
  todayISO: todayISO,
  nowISOSeconds: nowISOSeconds,
  runJob: runJob,
  cliMain: cliMain,
  SEED_PROFILES: SEED_PROFILES
};

global.LaserForge = LaserForge;
if (typeof module !== "undefined" && module.exports) {
  module.exports = LaserForge;
}
if (typeof require !== "undefined" && typeof module !== "undefined" &&
    require.main === module) {
  process.exit(cliMain(process.argv.slice(2)));
}

})(typeof window !== "undefined" ? window
   : (typeof globalThis !== "undefined" ? globalThis : this));
