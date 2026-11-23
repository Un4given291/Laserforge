#!/usr/bin/env python3
"""
LaserForge -- SVG -> GRBL gcode for Richard's CR-Laser Falcon Pro.

Piece 1: the engine core, flat work. Piece 2 (this build): rotary
emission on the mm-per-rotation model -- the wrap-direction (Y)
coordinates are scaled by mm_per_rotation / circumference, where
mm_per_rotation is a machine+chuck constant held in a JSON machine
profile (profiles.json), never in the job. A built-in calibration flow
(--calibrate-generate / --calibrate-apply) measures that constant from
one tape burn: true = used x intended / measured.

Python 3, standard library only (must run on Richard's PC as-is).

Pipeline: SVG load (units -> mm, transforms, curves flattened, Y flipped
so the burn matches the on-screen design) -> import cleanup guards
(duplicate dedupe, auto-join, unclosed-path warnings -- all reported,
never silent) -> layers of ordered ops (line / fill / cut) -> gcode.

Output dialect follows the known-good Falcon Design Space export: comment
header with bounds + mode + job statistics, G21 / M4, S on the 0..1000
scale, laser off (S0) around travel moves, S0 + M5 footer. Two start
modes: "absolute" (G90 workspace coordinates, design placed by offset on
the bed, footer returns to the origin) and the default "current"
(G91 relative moves anchored to wherever the head sits at job start --
Richard parks the head on a physical marker -- with the design's anchor
edge/centre on the head and a closing return to the start point). After
generating, the tool re-reads its own output, recomputes the emitted
bounds, and refuses to write the file if anything leaves the bed
(absolute) or the design span exceeds the bed size (current).
"""

import argparse
import datetime
import json
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

VERSION = "0.1.0"
FLATTEN_TOL = 0.05            # mm, curve flattening tolerance
CLOSE_TOL = 0.01              # mm: endpoints this close count as closed at load
JOIN_TOL = 0.05               # mm: auto-join / gap-close tolerance at cleanup
DEFAULT_BED = (400.0, 415.0)  # mm, Falcon Pro (TO CONFIRM on the machine)
RAPID_FEED = 6000.0           # mm/min, assumed rapid rate for time estimates
IN_MM = 25.4                  # exact inch -> mm
DEFAULT_PROFILES_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "profiles.json")


def die(msg, code=2):
    print("laserforge: error: %s" % msg, file=sys.stderr)
    sys.exit(code)


def fmt(v):
    """Format a coordinate with 3 decimals, avoiding '-0.000'."""
    s = "%.3f" % v
    return "0.000" if s == "-0.000" else s


def fmt_feed(v):
    return str(int(v)) if float(v) == int(v) else "%.1f" % v


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


# ---------------------------------------------------------------------------
# 2D affine transforms as (a, b, c, d, e, f):
#   x' = a*x + c*y + e
#   y' = b*x + d*y + f
# ---------------------------------------------------------------------------

IDENTITY = (1.0, 0.0, 0.0, 1.0, 0.0, 0.0)


def mat_mul(m, n):
    """Return m composed with n (n applied to the point first)."""
    return (m[0] * n[0] + m[2] * n[1],
            m[1] * n[0] + m[3] * n[1],
            m[0] * n[2] + m[2] * n[3],
            m[1] * n[2] + m[3] * n[3],
            m[0] * n[4] + m[2] * n[5] + m[4],
            m[1] * n[4] + m[3] * n[5] + m[5])


def mat_apply(m, p):
    return (m[0] * p[0] + m[2] * p[1] + m[4],
            m[1] * p[0] + m[3] * p[1] + m[5])


def mat_scale_factor(m):
    return max(math.hypot(m[0], m[1]), math.hypot(m[2], m[3]), 1e-9)


def parse_transform(text):
    """Parse an SVG transform attribute into a matrix."""
    mat = IDENTITY
    if not text:
        return mat
    for name, args in re.findall(
            r"(matrix|translate|scale|rotate|skewX|skewY)\s*\(([^)]*)\)", text):
        vals = [float(v) for v in re.split(r"[\s,]+", args.strip()) if v != ""]
        if name == "matrix" and len(vals) == 6:
            t = tuple(vals)
        elif name == "translate":
            t = (1.0, 0.0, 0.0, 1.0, vals[0], vals[1] if len(vals) > 1 else 0.0)
        elif name == "scale":
            t = (vals[0], 0.0, 0.0, vals[1] if len(vals) > 1 else vals[0], 0.0, 0.0)
        elif name == "rotate":
            th = math.radians(vals[0])
            rot = (math.cos(th), math.sin(th), -math.sin(th), math.cos(th), 0.0, 0.0)
            if len(vals) >= 3:
                cx, cy = vals[1], vals[2]
                t = mat_mul(mat_mul((1.0, 0.0, 0.0, 1.0, cx, cy), rot),
                            (1.0, 0.0, 0.0, 1.0, -cx, -cy))
            else:
                t = rot
        elif name == "skewX":
            t = (1.0, 0.0, math.tan(math.radians(vals[0])), 1.0, 0.0, 0.0)
        elif name == "skewY":
            t = (1.0, math.tan(math.radians(vals[0])), 0.0, 1.0, 0.0, 0.0)
        else:
            continue
        mat = mat_mul(mat, t)
    return mat


# ---------------------------------------------------------------------------
# SVG lengths / units
# ---------------------------------------------------------------------------

UNIT_TO_MM = {
    "mm": 1.0,
    "cm": 10.0,
    "in": 25.4,
    "pt": 25.4 / 72.0,
    "pc": 25.4 / 6.0,
    "px": 25.4 / 96.0,
    "": 25.4 / 96.0,   # bare user units are px at 96 dpi
}

_LENGTH_RE = re.compile(r"^\s*([+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?)\s*([a-z%]*)\s*$")


def parse_length(text):
    """Parse an SVG length -> (value, unit) or None."""
    if text is None:
        return None
    m = _LENGTH_RE.match(text)
    if not m:
        return None
    return (float(m.group(1)), m.group(2))


def length_to_mm(text):
    parsed = parse_length(text)
    if parsed is None:
        return None
    value, unit = parsed
    if unit == "%":
        return None
    return value * UNIT_TO_MM.get(unit, 25.4 / 96.0)


# ---------------------------------------------------------------------------
# Curve flattening (local coordinates; tolerance in the same units)
# ---------------------------------------------------------------------------

def _dist_to_line(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    chord = math.hypot(dx, dy)
    if chord < 1e-12:
        return dist(p, a)
    return abs((p[0] - a[0]) * dy - (p[1] - a[1]) * dx) / chord


def _mid(a, b):
    return ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)


def flatten_cubic(p0, p1, p2, p3, tol, out):
    """Append flattened points of a cubic Bezier (excluding p0) to out."""
    stack = [(p0, p1, p2, p3, 0)]
    while stack:
        a, b, c, d, depth = stack.pop()
        if depth >= 20 or max(_dist_to_line(b, a, d), _dist_to_line(c, a, d)) <= tol:
            out.append(d)
        else:
            ab, bc, cd = _mid(a, b), _mid(b, c), _mid(c, d)
            abc, bcd = _mid(ab, bc), _mid(bc, cd)
            abcd = _mid(abc, bcd)
            stack.append((abcd, bcd, cd, d, depth + 1))
            stack.append((a, ab, abc, abcd, depth + 1))


def flatten_quad(p0, q, p2, tol, out):
    """Append flattened points of a quadratic Bezier (excluding p0) to out."""
    b = (p0[0] + (2.0 / 3.0) * (q[0] - p0[0]), p0[1] + (2.0 / 3.0) * (q[1] - p0[1]))
    c = (p2[0] + (2.0 / 3.0) * (q[0] - p2[0]), p2[1] + (2.0 / 3.0) * (q[1] - p2[1]))
    flatten_cubic(p0, b, c, p2, tol, out)


def arc_to_points(p0, rx, ry, rot_deg, large_arc, sweep, p1, tol):
    """SVG endpoint arc -> points excluding start, including end.

    Implements the SVG 1.1 F.6.5 endpoint-to-center conversion.
    """
    if rx == 0 or ry == 0:
        return [p1]
    if p0 == p1:
        return []
    rx, ry = abs(rx), abs(ry)
    phi = math.radians(rot_deg)
    cos_p, sin_p = math.cos(phi), math.sin(phi)
    dx, dy = (p0[0] - p1[0]) / 2.0, (p0[1] - p1[1]) / 2.0
    x1p = cos_p * dx + sin_p * dy
    y1p = -sin_p * dx + cos_p * dy
    lam = (x1p * x1p) / (rx * rx) + (y1p * y1p) / (ry * ry)
    if lam > 1.0:
        s = math.sqrt(lam)
        rx, ry = rx * s, ry * s
    num = rx * rx * ry * ry - rx * rx * y1p * y1p - ry * ry * x1p * x1p
    den = rx * rx * y1p * y1p + ry * ry * x1p * x1p
    co = math.sqrt(max(0.0, num / den)) if den != 0 else 0.0
    if large_arc == sweep:
        co = -co
    cxp = co * rx * y1p / ry
    cyp = -co * ry * x1p / rx
    cx = cos_p * cxp - sin_p * cyp + (p0[0] + p1[0]) / 2.0
    cy = sin_p * cxp + cos_p * cyp + (p0[1] + p1[1]) / 2.0

    def angle(ux, uy, vx, vy):
        d = (ux * vx + uy * vy) / (math.hypot(ux, uy) * math.hypot(vx, vy))
        a = math.acos(clamp(d, -1.0, 1.0))
        return -a if ux * vy - uy * vx < 0 else a

    theta1 = angle(1.0, 0.0, (x1p - cxp) / rx, (y1p - cyp) / ry)
    dtheta = angle((x1p - cxp) / rx, (y1p - cyp) / ry,
                   (-x1p - cxp) / rx, (-y1p - cyp) / ry)
    if not sweep and dtheta > 0:
        dtheta -= 2.0 * math.pi
    elif sweep and dtheta < 0:
        dtheta += 2.0 * math.pi
    r_max = max(rx, ry)
    step = 2.0 * math.acos(clamp(1.0 - tol / r_max, -1.0, 1.0)) if r_max > tol else math.pi / 8.0
    n = max(1, int(math.ceil(abs(dtheta) / max(step, 1e-6))))
    pts = []
    for i in range(1, n + 1):
        t = theta1 + dtheta * i / n
        ex, ey = rx * math.cos(t), ry * math.sin(t)
        pts.append((cos_p * ex - sin_p * ey + cx, sin_p * ex + cos_p * ey + cy))
    pts[-1] = p1
    return pts


def ellipse_points(cx, cy, rx, ry, tol):
    """Full ellipse as a closed polyline (first point repeated at the end)."""
    r_max = max(rx, ry)
    if r_max <= 0:
        return []
    step = 2.0 * math.acos(clamp(1.0 - tol / r_max, -1.0, 1.0)) if r_max > tol else math.pi / 4.0
    n = max(8, int(math.ceil(2.0 * math.pi / max(step, 1e-6))))
    pts = [(cx + rx * math.cos(2.0 * math.pi * i / n),
            cy + ry * math.sin(2.0 * math.pi * i / n)) for i in range(n)]
    pts.append(pts[0])
    return pts


# ---------------------------------------------------------------------------
# SVG path data parsing (scanner-based; arc flags read as single digits so
# packed forms like "0 01150 90" tokenise the way the SVG spec intends)
# ---------------------------------------------------------------------------

_NUM_RE = re.compile(r"[+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?")


class _Scanner:
    def __init__(self, text):
        self.s = text or ""
        self.i = 0
        self.n = len(self.s)

    def skip(self):
        while self.i < self.n and self.s[self.i] in " \t\r\n,":
            self.i += 1

    def eof(self):
        self.skip()
        return self.i >= self.n

    def peek_cmd(self):
        self.skip()
        if self.i < self.n and self.s[self.i].isalpha():
            return self.s[self.i]
        return None

    def read_cmd(self):
        self.skip()
        c = self.s[self.i]
        self.i += 1
        return c

    def read_num(self):
        self.skip()
        m = _NUM_RE.match(self.s, self.i)
        if not m:
            raise ValueError("expected number at %d in path data" % self.i)
        self.i = m.end()
        return float(m.group())

    def read_flag(self):
        self.skip()
        if self.i >= self.n or self.s[self.i] not in "01":
            raise ValueError("expected arc flag at %d in path data" % self.i)
        v = int(self.s[self.i])
        self.i += 1
        return v


def parse_path(d, tol):
    """Parse SVG path data -> list of (points, closed) in local coordinates.

    Closed subpaths include the start point again as their last point.
    """
    sc = _Scanner(d)
    subpaths = []
    pts = None
    cur = (0.0, 0.0)
    start = (0.0, 0.0)
    cmd = None
    prev_cubic = None     # previous cubic's second control point
    prev_quad = None      # previous quadratic's control point

    def flush():
        nonlocal pts
        if pts:
            subpaths.append((pts, False))
        pts = None

    def ensure_pts():
        nonlocal pts
        if pts is None:
            pts = [cur]

    while not sc.eof():
        letter = sc.peek_cmd()
        if letter is not None:
            cmd = sc.read_cmd()
        else:
            if cmd is None:
                raise ValueError("path data must start with a command")
            if cmd in ("Z", "z"):
                raise ValueError("unexpected arguments after Z")
            if cmd in ("M", "m"):
                cmd = "L" if cmd.isupper() else "l"
        upper = cmd.upper()
        rel = cmd.islower()

        if upper == "Z":
            if pts:
                if dist(pts[-1], start) > 1e-12:
                    pts.append(start)
                subpaths.append((pts, True))
                pts = None
            cur = start
            prev_cubic = prev_quad = None
            cmd = None
            continue

        if upper == "M":
            x, y = sc.read_num(), sc.read_num()
            if rel:
                x, y = x + cur[0], y + cur[1]
            flush()
            cur = start = (x, y)
            pts = [cur]
            prev_cubic = prev_quad = None
            cmd = "L" if not rel else "l"
            continue

        ensure_pts()

        if upper == "L":
            x, y = sc.read_num(), sc.read_num()
            if rel:
                x, y = x + cur[0], y + cur[1]
            cur = (x, y)
            pts.append(cur)
            prev_cubic = prev_quad = None
        elif upper == "H":
            x = sc.read_num()
            if rel:
                x += cur[0]
            cur = (x, cur[1])
            pts.append(cur)
            prev_cubic = prev_quad = None
        elif upper == "V":
            y = sc.read_num()
            if rel:
                y += cur[1]
            cur = (cur[0], y)
            pts.append(cur)
            prev_cubic = prev_quad = None
        elif upper == "C":
            x1, y1 = sc.read_num(), sc.read_num()
            x2, y2 = sc.read_num(), sc.read_num()
            x, y = sc.read_num(), sc.read_num()
            if rel:
                x1, y1 = x1 + cur[0], y1 + cur[1]
                x2, y2 = x2 + cur[0], y2 + cur[1]
                x, y = x + cur[0], y + cur[1]
            flat = []
            flatten_cubic(cur, (x1, y1), (x2, y2), (x, y), tol, flat)
            pts.extend(flat)
            prev_cubic, prev_quad = (x2, y2), None
            cur = (x, y)
        elif upper == "S":
            x2, y2 = sc.read_num(), sc.read_num()
            x, y = sc.read_num(), sc.read_num()
            if rel:
                x2, y2 = x2 + cur[0], y2 + cur[1]
                x, y = x + cur[0], y + cur[1]
            if prev_cubic is not None:
                x1 = 2.0 * cur[0] - prev_cubic[0]
                y1 = 2.0 * cur[1] - prev_cubic[1]
            else:
                x1, y1 = cur
            flat = []
            flatten_cubic(cur, (x1, y1), (x2, y2), (x, y), tol, flat)
            pts.extend(flat)
            prev_cubic, prev_quad = (x2, y2), None
            cur = (x, y)
        elif upper == "Q":
            x1, y1 = sc.read_num(), sc.read_num()
            x, y = sc.read_num(), sc.read_num()
            if rel:
                x1, y1 = x1 + cur[0], y1 + cur[1]
                x, y = x + cur[0], y + cur[1]
            flat = []
            flatten_quad(cur, (x1, y1), (x, y), tol, flat)
            pts.extend(flat)
            prev_cubic, prev_quad = None, (x1, y1)
            cur = (x, y)
        elif upper == "T":
            x, y = sc.read_num(), sc.read_num()
            if rel:
                x, y = x + cur[0], y + cur[1]
            if prev_quad is not None:
                x1 = 2.0 * cur[0] - prev_quad[0]
                y1 = 2.0 * cur[1] - prev_quad[1]
            else:
                x1, y1 = cur
            flat = []
            flatten_quad(cur, (x1, y1), (x, y), tol, flat)
            pts.extend(flat)
            prev_cubic, prev_quad = None, (x1, y1)
            cur = (x, y)
        elif upper == "A":
            rx, ry = sc.read_num(), sc.read_num()
            rot = sc.read_num()
            large = sc.read_flag()
            sweep = sc.read_flag()
            x, y = sc.read_num(), sc.read_num()
            if rel:
                x, y = x + cur[0], y + cur[1]
            pts.extend(arc_to_points(cur, rx, ry, rot, large, sweep, (x, y), tol))
            cur = (x, y)
            prev_cubic = prev_quad = None
        else:
            raise ValueError("unsupported path command %r" % cmd)

    flush()
    return subpaths


# ---------------------------------------------------------------------------
# SVG shape elements -> (points, closed) in local coordinates
# ---------------------------------------------------------------------------

def _attr_num(elem, name, default=0.0):
    parsed = parse_length(elem.get(name))
    return parsed[0] if parsed else default


def _rect_subpaths(elem, tol):
    x, y = _attr_num(elem, "x"), _attr_num(elem, "y")
    w, h = _attr_num(elem, "width"), _attr_num(elem, "height")
    if w <= 0 or h <= 0:
        return []
    if elem.get("rx") is None and elem.get("ry") is None:
        return [([(x, y), (x + w, y), (x + w, y + h), (x, y + h), (x, y)], True)]
    rxv = _attr_num(elem, "rx") if elem.get("rx") is not None else _attr_num(elem, "ry")
    ryv = _attr_num(elem, "ry") if elem.get("ry") is not None else rxv
    rxv, ryv = min(rxv, w / 2.0), min(ryv, h / 2.0)
    pts = [(x + rxv, y), (x + w - rxv, y)]
    pts.extend(arc_to_points(pts[-1], rxv, ryv, 0, 0, 1, (x + w, y + ryv), tol))
    pts.append((x + w, y + h - ryv))
    pts.extend(arc_to_points(pts[-1], rxv, ryv, 0, 0, 1, (x + w - rxv, y + h), tol))
    pts.append((x + rxv, y + h))
    pts.extend(arc_to_points(pts[-1], rxv, ryv, 0, 0, 1, (x, y + h - ryv), tol))
    pts.append((x, y + ryv))
    pts.extend(arc_to_points(pts[-1], rxv, ryv, 0, 0, 1, (x + rxv, y), tol))
    return [(pts, True)]


def _points_attr(elem):
    raw = elem.get("points") or ""
    vals = [float(v) for v in re.split(r"[\s,]+", raw.strip()) if v != ""]
    return [(vals[i], vals[i + 1]) for i in range(0, len(vals) - 1, 2)]


def shape_subpaths(elem, tag, tol):
    if tag == "path":
        return parse_path(elem.get("d") or "", tol)
    if tag == "rect":
        return _rect_subpaths(elem, tol)
    if tag == "circle":
        pts = ellipse_points(_attr_num(elem, "cx"), _attr_num(elem, "cy"),
                             _attr_num(elem, "r"), _attr_num(elem, "r"), tol)
        return [(pts, True)] if pts else []
    if tag == "ellipse":
        pts = ellipse_points(_attr_num(elem, "cx"), _attr_num(elem, "cy"),
                             _attr_num(elem, "rx"), _attr_num(elem, "ry"), tol)
        return [(pts, True)] if pts else []
    if tag == "line":
        return [([(_attr_num(elem, "x1"), _attr_num(elem, "y1")),
                  (_attr_num(elem, "x2"), _attr_num(elem, "y2"))], False)]
    if tag == "polyline":
        pts = _points_attr(elem)
        return [(pts, False)] if len(pts) >= 2 else []
    if tag == "polygon":
        pts = _points_attr(elem)
        if len(pts) >= 2:
            if pts[0] != pts[-1]:
                pts = pts + [pts[0]]
            return [(pts, True)]
        return []
    return []


_SHAPE_TAGS = {"path", "rect", "circle", "ellipse", "line", "polyline", "polygon"}
_SKIP_TAGS = {"defs", "title", "desc", "metadata", "style", "script"}


def _is_hidden(elem):
    if elem.get("display") == "none" or elem.get("visibility") == "hidden":
        return True
    style = (elem.get("style") or "").replace(" ", "")
    return "display:none" in style or "visibility:hidden" in style


class Shape:
    """One flattened polyline in document mm (after the Y flip).

    `index` is the 0-based document order; `label` is the element id when
    the SVG provides one, else "shape-<index+1>". `elem_id` / `group_ids`
    are kept for layer selection.
    """
    __slots__ = ("points", "closed", "label", "index", "elem_id", "group_ids")

    def __init__(self, points, closed, label, index, elem_id, group_ids):
        self.points = points
        self.closed = closed
        self.label = label
        self.index = index
        self.elem_id = elem_id
        self.group_ids = group_ids


def _root_transform(root):
    """(matrix mapping user units -> mm, document height in mm or None)."""
    vb = None
    if root.get("viewBox"):
        parts = [float(v) for v in re.split(r"[\s,]+", root.get("viewBox").strip()) if v != ""]
        if len(parts) == 4:
            vb = parts
    w_mm = length_to_mm(root.get("width"))
    h_mm = length_to_mm(root.get("height"))
    if vb is None:
        # No viewBox: user units are px at 96 dpi; the document height in
        # mm comes from the height attribute if it carries one.
        s = 25.4 / 96.0
        return (s, 0.0, 0.0, s, 0.0, 0.0), h_mm
    minx, miny, vbw, vbh = vb
    if w_mm is None:
        w_mm = vbw * 25.4 / 96.0
    if h_mm is None:
        h_mm = vbh * 25.4 / 96.0
    if vbw <= 0 or vbh <= 0:
        return IDENTITY, h_mm
    par = (root.get("preserveAspectRatio") or "xMidYMid meet").strip()
    if par == "none":
        sx, sy = w_mm / vbw, h_mm / vbh
        return (sx, 0.0, 0.0, sy, -minx * sx, -miny * sy), h_mm
    s = min(w_mm / vbw, h_mm / vbh) if "meet" in par else max(w_mm / vbw, h_mm / vbh)
    ax = (w_mm - vbw * s) / 2.0 if "xMid" in par else (w_mm - vbw * s if "xMax" in par else 0.0)
    ay = (h_mm - vbh * s) / 2.0 if "YMid" in par else (h_mm - vbh * s if "YMax" in par else 0.0)
    return (s, 0.0, 0.0, s, ax - minx * s, ay - miny * s), h_mm


def load_svg(path):
    """Load an SVG -> (shapes, report lines). Shapes are in document mm,
    Y flipped so the burn matches the on-screen design (bed origin at the
    lower-left of the design)."""
    try:
        tree = ET.parse(path)
    except ET.ParseError as exc:
        die("cannot parse %s as XML: %s" % (path, exc))
    root = tree.getroot()
    if root.tag.split("}")[-1] != "svg":
        die("%s: root element is not <svg>" % path)
    base, doc_h = _root_transform(root)
    shapes = []
    dropped = [0]

    def add(pts, closed, elem, group_ids):
        clean = []
        for p in pts:
            if not clean or dist(p, clean[-1]) > 1e-9:
                clean.append(p)
        if len(clean) < 2 or sum(dist(clean[i], clean[i + 1])
                                 for i in range(len(clean) - 1)) <= 0:
            dropped[0] += 1
            return
        if not closed and len(clean) >= 3 and dist(clean[0], clean[-1]) <= CLOSE_TOL:
            clean[-1] = clean[0]
            closed = True
        idx = len(shapes)
        elem_id = elem.get("id")
        shapes.append(Shape(clean, closed, elem_id or "shape-%d" % (idx + 1),
                            idx, elem_id, list(group_ids)))

    def walk(elem, mat, group_ids):
        tag = elem.tag.split("}")[-1]
        if tag in _SKIP_TAGS or _is_hidden(elem):
            return
        m = mat_mul(mat, parse_transform(elem.get("transform")))
        if tag in _SHAPE_TAGS:
            tol_local = FLATTEN_TOL / mat_scale_factor(m)
            try:
                subs = shape_subpaths(elem, tag, tol_local)
            except ValueError as exc:
                die("%s: <%s> %s: %s" % (path, tag, elem.get("id") or "", exc))
            for pts, closed in subs:
                add([mat_apply(m, p) for p in pts], closed, elem, group_ids)
            return
        child_groups = group_ids + [elem.get("id")] if elem.get("id") else group_ids
        for child in elem:
            walk(child, m, child_groups)

    walk(root, base, [])

    # Flip Y: SVG is Y-down, the bed is Y-up. Flip around the document
    # height when known, else around the shapes' own top edge.
    if shapes:
        if doc_h is not None:
            flip_ref = doc_h
        else:
            flip_ref = max(p[1] for sh in shapes for p in sh.points)
        for sh in shapes:
            sh.points = [(x, flip_ref - y) for (x, y) in sh.points]

    report = ["import: loaded %d shape(s) from %s" % (len(shapes), os.path.basename(path))]
    if dropped[0]:
        report.append("import: dropped %d degenerate shape(s) "
                      "(fewer than 2 distinct points / zero length)" % dropped[0])
    return shapes, report


# ---------------------------------------------------------------------------
# Import cleanup guards -- every action is reported, never silent
# ---------------------------------------------------------------------------

def _canonical_key(sh):
    pts = [(round(x, 2), round(y, 2)) for (x, y) in sh.points]
    if sh.closed and len(pts) > 1 and pts[0] == pts[-1]:
        pts = pts[:-1]
    if sh.closed and 2 <= len(pts) <= 400:
        best = None
        for seq in (pts, pts[::-1]):
            for k in range(len(seq)):
                rot = tuple(seq[k:] + seq[:k])
                if best is None or rot < best:
                    best = rot
        return (True, best)
    return (sh.closed, min(tuple(pts), tuple(pts[::-1])))


def _try_join(a, b):
    """If an endpoint of a is within JOIN_TOL of an endpoint of b, return
    (merged points, gap); else None."""
    combos = [
        (a.points[-1], b.points[0], lambda: a.points + b.points),
        (a.points[-1], b.points[-1], lambda: a.points + b.points[::-1]),
        (a.points[0], b.points[-1], lambda: b.points + a.points),
        (a.points[0], b.points[0], lambda: b.points[::-1] + a.points),
    ]
    for pa, pb, build in combos:
        gap = dist(pa, pb)
        if gap <= JOIN_TOL:
            merged = build()
            clean = []
            for p in merged:
                if not clean or dist(p, clean[-1]) > 1e-9:
                    clean.append(p)
            return clean, gap
    return None


def cleanup_shapes(shapes, report):
    # 1) duplicate-path dedupe
    seen = {}
    kept = []
    for sh in shapes:
        key = _canonical_key(sh)
        if key in seen:
            report.append("import: removed duplicate path '%s' "
                          "(identical geometry to '%s')" % (sh.label, seen[key]))
        else:
            seen[key] = sh.label
            kept.append(sh)
    shapes = kept

    # 2) auto-join touching segments + close small gaps (loop: a join can
    #    create a shape whose own ends now meet)
    changed = True
    while changed:
        changed = False
        for sh in shapes:
            if not sh.closed and len(sh.points) >= 3:
                gap = dist(sh.points[0], sh.points[-1])
                if gap <= JOIN_TOL:
                    sh.points[-1] = sh.points[0]
                    sh.closed = True
                    report.append("import: closed path '%s' (endpoints %.3f mm apart)"
                                  % (sh.label, gap))
                    changed = True
        for i in range(len(shapes)):
            if shapes[i].closed:
                continue
            merged_done = False
            for j in range(i + 1, len(shapes)):
                if shapes[j].closed:
                    continue
                res = _try_join(shapes[i], shapes[j])
                if res:
                    merged, gap = res
                    report.append("import: auto-joined '%s' + '%s' "
                                  "(endpoints %.3f mm apart)"
                                  % (shapes[i].label, shapes[j].label, gap))
                    shapes[i].points = merged
                    shapes[i].label = shapes[i].label + "+" + shapes[j].label
                    del shapes[j]
                    changed = True
                    merged_done = True
                    break
            if merged_done:
                break

    # 3) unclosed-path warnings
    for sh in shapes:
        if not sh.closed:
            report.append("import: WARNING path '%s' is not closed -- usable "
                          "for line/cut, excluded from fill" % sh.label)
    return shapes


# ---------------------------------------------------------------------------
# Hatch fill (even-odd across all of a layer's closed shapes together, so
# a shape inside a shape leaves the ring between them filled)
# ---------------------------------------------------------------------------

def hatch_segments(polys, spacing, angle_deg, bidirectional=True):
    """Return a list of ((x0, y0), (x1, y1)) hatch segments.

    Scanlines sit at spacing/2 offsets from the lowest edge; with
    bidirectional scanning each line's segments run in the opposite
    direction to the previous line's (serpentine, less travel).
    """
    if spacing <= 0:
        die("fill spacing must be > 0")
    ang = math.radians(angle_deg)
    ca, sa = math.cos(ang), math.sin(ang)

    def rot(p):    # rotate by -angle: scan direction becomes +X
        return (ca * p[0] + sa * p[1], -sa * p[0] + ca * p[1])

    def unrot(p):  # rotate back by +angle
        return (ca * p[0] - sa * p[1], sa * p[0] + ca * p[1])

    edges = []
    ys = []
    for poly in polys:
        rp = [rot(p) for p in poly]
        ys.extend(p[1] for p in rp)
        for i in range(len(rp)):
            x0, y0 = rp[i]
            x1, y1 = rp[(i + 1) % len(rp)]
            if y0 != y1:
                edges.append((x0, y0, x1, y1))
    if not edges:
        return []
    ymin, ymax = min(ys), max(ys)
    segments = []
    k = 0
    y = ymin + spacing * 0.5
    while y < ymax - 1e-9:
        xs = []
        for (x0, y0, x1, y1) in edges:
            if (y0 <= y < y1) or (y1 <= y < y0):
                xs.append(x0 + (y - y0) * (x1 - x0) / (y1 - y0))
        xs.sort()
        line_segs = []
        for i in range(0, len(xs) - 1, 2):
            if xs[i + 1] - xs[i] > 1e-9:
                line_segs.append((unrot((xs[i], y)), unrot((xs[i + 1], y))))
        if bidirectional and k % 2 == 1:
            line_segs = [(b, a) for (a, b) in reversed(line_segs)]
        segments.extend(line_segs)
        k += 1
        y += spacing
    return segments


# ---------------------------------------------------------------------------
# Job model: layers of ordered ops (sub-layers)
# ---------------------------------------------------------------------------

OP_DEFAULTS = {
    "line": {"speed": 1000.0, "power": 50.0, "passes": 1},
    "cut": {"speed": 300.0, "power": 90.0, "passes": 1},
    "fill": {"speed": 1500.0, "power": 50.0, "passes": 1,
             "spacing": 0.1, "angle": 0.0, "bidirectional": True,
             "outline": False},
}
_OP_KEYS = {"speed", "power", "passes", "spacing", "angle", "bidirectional", "outline"}


def normalize_op(raw, layer_name):
    otype = raw.get("op") or raw.get("mode") or "line"
    if otype not in OP_DEFAULTS:
        die("layer '%s': unknown op %r (want line, fill or cut)" % (layer_name, otype))
    op = dict(OP_DEFAULTS[otype])
    for key in _OP_KEYS:
        if key in raw:
            op[key] = raw[key]
    op["type"] = otype
    op["speed"] = float(op["speed"])
    op["power"] = float(op["power"])
    op["passes"] = int(op["passes"])
    if op["speed"] <= 0:
        die("layer '%s': speed must be > 0" % layer_name)
    if not 0.0 <= op["power"] <= 100.0:
        die("layer '%s': power must be between 0 and 100 (got %s)"
            % (layer_name, op["power"]))
    if op["passes"] < 1:
        die("layer '%s': passes must be >= 1" % layer_name)
    if otype == "fill":
        op["spacing"] = float(op["spacing"])
        op["angle"] = float(op["angle"])
        if op["spacing"] <= 0:
            die("layer '%s': fill spacing must be > 0" % layer_name)
    return op


def normalize_layer(raw, idx):
    name = raw.get("name") or "layer-%d" % (idx + 1)
    if "ops" in raw:
        if not raw["ops"]:
            die("layer '%s': ops list is empty" % name)
        ops = [normalize_op(o, name) for o in raw["ops"]]
    else:
        ops = [normalize_op(raw, name)]
    return {"name": name,
            "output": bool(raw.get("output", True)),
            "select": raw.get("select"),
            "ops": ops}


def resolve_select(sel, shapes, claimed):
    """Geometry assignment (see README): a layer's `select` list names
    shapes by element id, group id, or label ("shape-<n>"), or by 0-based
    document index (integer). A layer with no `select` claims every shape
    not claimed by an earlier layer."""
    if sel is None:
        return [sh for sh in shapes if sh.index not in claimed]
    if not isinstance(sel, list):
        sel = [sel]
    out = []
    for item in sel:
        if isinstance(item, int):
            match = [sh for sh in shapes if sh.index == item]
        else:
            match = [sh for sh in shapes
                     if sh.label == str(item) or sh.elem_id == str(item)
                     or str(item) in sh.group_ids]
        for sh in match:
            if sh not in out:
                out.append(sh)
    return out


class Stroke:
    __slots__ = ("points", "feed", "s", "power", "layer", "op", "pass_no", "passes")

    def __init__(self, points, feed, s, power, layer, op, pass_no, passes):
        self.points = points
        self.feed = feed
        self.s = s
        self.power = power
        self.layer = layer
        self.op = op
        self.pass_no = pass_no
        self.passes = passes


def build_strokes(job, shapes, report):
    strokes = []
    claimed = set()
    for layer in job["layers"]:
        if not layer["output"]:
            report.append("layer '%s': output disabled -- skipped" % layer["name"])
            continue
        sel = resolve_select(layer["select"], shapes, claimed)
        if not sel:
            report.append("layer '%s': WARNING no geometry selected -- skipped"
                          % layer["name"])
            continue
        for sh in sel:
            claimed.add(sh.index)
        report.append("layer '%s': %d shape(s), %d op(s)"
                      % (layer["name"], len(sel), len(layer["ops"])))
        for op in layer["ops"]:
            sval = int(round(op["power"] * 10.0))
            for pno in range(1, op["passes"] + 1):
                if op["type"] in ("line", "cut"):
                    for sh in sel:
                        pts = list(sh.points)
                        if sh.closed and pts[0] != pts[-1]:
                            pts.append(pts[0])
                        strokes.append(Stroke(pts, op["speed"], sval, op["power"],
                                              layer["name"], op["type"], pno,
                                              op["passes"]))
                else:  # fill
                    closed_shapes = [sh for sh in sel if sh.closed]
                    n_open = len(sel) - len(closed_shapes)
                    if n_open:
                        report.append("layer '%s': fill skipped %d open path(s) "
                                      "(fill needs closed paths)"
                                      % (layer["name"], n_open))
                    polys = []
                    for sh in closed_shapes:
                        pts = list(sh.points)
                        if pts and pts[0] == pts[-1]:
                            pts = pts[:-1]
                        if len(pts) >= 3:
                            polys.append(pts)
                    if polys:
                        for (a, b) in hatch_segments(polys, op["spacing"],
                                                     op["angle"],
                                                     op["bidirectional"]):
                            strokes.append(Stroke([a, b], op["speed"], sval,
                                                  op["power"], layer["name"],
                                                  op["type"], pno, op["passes"]))
                        if op["outline"]:
                            for pts in polys:
                                strokes.append(Stroke(pts + [pts[0]], op["speed"],
                                                      sval, op["power"],
                                                      layer["name"], op["type"],
                                                      pno, op["passes"]))
    return strokes


# ---------------------------------------------------------------------------
# Job statistics, rendering, self-verification
#
# Time estimate model (also in README): burn time = sum over burn segments
# of length / that op's feed rate; travel time = sum over travel moves of
# length / RAPID_FEED. No acceleration model -- it is an estimate, and it
# errs on the low side for hatch-heavy jobs with many short segments.
# ---------------------------------------------------------------------------

def analyze(strokes):
    burn = travel = minutes = 0.0
    pos = (0.0, 0.0)
    bounds = None

    def grow(p):
        nonlocal bounds
        if bounds is None:
            bounds = [p[0], p[1], p[0], p[1]]
        else:
            bounds[0] = min(bounds[0], p[0])
            bounds[1] = min(bounds[1], p[1])
            bounds[2] = max(bounds[2], p[0])
            bounds[3] = max(bounds[3], p[1])

    for st in strokes:
        d = dist(pos, st.points[0])
        travel += d
        minutes += d / RAPID_FEED
        prev = st.points[0]
        grow(prev)
        for p in st.points[1:]:
            seg = dist(prev, p)
            burn += seg
            minutes += seg / st.feed
            grow(p)
            prev = p
        pos = st.points[-1]
    d = dist(pos, (0.0, 0.0))     # final return home (bed origin, or the
    # start point in current mode -- the frame origin in both cases)
    travel += d
    minutes += d / RAPID_FEED
    return {"burn": burn, "travel": travel, "minutes": minutes, "bounds": bounds}


def _rotary_header_lines(meta):
    """Extra header lines for rotary jobs (empty for flat work)."""
    r = meta.get("rotary")
    if not r:
        return []
    lines = [
        "; Rotary profile: %s (%s)" % (r["profile_key"], r["profile_name"]),
        "; mm per rotation: %s mm" % fmt(r["mm_per_rotation"]),
        "; Profile status: %s" % r["status"],
        "; Circumference: %s mm (%s in)"
        % (fmt(r["circumference_mm"]), fmt(r["circumference_mm"] / IN_MM)),
        "; Wrap scale: Y x %.6f (mm_per_rotation / circumference)" % r["factor"],
    ]
    if meta.get("calibration"):
        c = meta["calibration"]
        lines.append(
            "; Calibration line: intended wrap arc %s mm (%s in) at "
            "mm_per_rotation %s -- burn on tape, measure the arc, then "
            "run --calibrate-apply"
            % (fmt(c["test_arc_mm"]), fmt(c["test_arc_mm"] / IN_MM),
               fmt(r["mm_per_rotation"])))
    return lines


def render_gcode(strokes, stats, meta):
    b = stats["bounds"]
    lines = [
        "; LaserForge %s -- SVG -> GRBL gcode" % VERSION,
        "; Source: %s" % meta["source"],
        "; Mode: %s" % meta["mode"],
        "; Bounds: X%s Y%s to X%s Y%s" % (fmt(b[0]), fmt(b[1]), fmt(b[2]), fmt(b[3])),
        "; Burn distance: %.1f mm" % stats["burn"],
        "; Travel distance: %.1f mm" % stats["travel"],
        "; Estimated time: %.2f min" % stats["minutes"],
        "; Layers: %s" % ", ".join(meta["layer_names"]),
        "G21",
        "G90",
        "M4",
        "S0",
    ]
    lines[3:3] = _rotary_header_lines(meta)
    group = None
    for st in strokes:
        key = (st.layer, st.op, st.pass_no)
        if key != group:
            group = key
            lines.append("; Layer '%s' -- %s, %s mm/min, %g%% (S%d), pass %d/%d"
                         % (st.layer, st.op, fmt_feed(st.feed), st.power, st.s,
                            st.pass_no, st.passes))
        lines.append("G0 X%s Y%s" % (fmt(st.points[0][0]), fmt(st.points[0][1])))
        first = True
        prev = st.points[0]
        for p in st.points[1:]:
            if dist(prev, p) < 1e-12:
                continue
            if first:
                lines.append("G1 X%s Y%s F%s S%d"
                             % (fmt(p[0]), fmt(p[1]), fmt_feed(st.feed), st.s))
                first = False
            else:
                lines.append("G1 X%s Y%s" % (fmt(p[0]), fmt(p[1])))
            prev = p
        lines.append("S0")
    lines += ["; End of job", "S0", "M5", "G0 X0.000 Y0.000"]
    return "\n".join(lines) + "\n"


def verify_gcode(text, bed_w, bed_h):
    """Re-read the generated output: recompute the burn bounds from the
    G1 moves made while S > 0, and flag any emitted position (burn or
    travel) that leaves the bed."""
    x = y = 0.0
    s = 0
    bounds = None
    problems = []
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith(";"):
            continue
        cmd = None
        nx, ny, ns = x, y, s
        for tok in line.split():
            head = tok[0]
            if head == "G":
                cmd = tok
            elif head == "X":
                nx = float(tok[1:])
            elif head == "Y":
                ny = float(tok[1:])
            elif head == "S":
                ns = int(float(tok[1:]))
        if cmd in ("G0", "G1"):
            if cmd == "G1" and ns > 0:
                for p in ((x, y), (nx, ny)):
                    if bounds is None:
                        bounds = [p[0], p[1], p[0], p[1]]
                    else:
                        bounds[0] = min(bounds[0], p[0])
                        bounds[1] = min(bounds[1], p[1])
                        bounds[2] = max(bounds[2], p[0])
                        bounds[3] = max(bounds[3], p[1])
            x, y = nx, ny
            if x < -0.001 or y < -0.001 or x > bed_w + 0.001 or y > bed_h + 0.001:
                problems.append("position X%s Y%s leaves the bed (%.0f x %.0f mm)"
                                % (fmt(x), fmt(y), bed_w, bed_h))
        s = ns
    return bounds, problems


# ---------------------------------------------------------------------------
# Current-position start mode: G91 relative emission anchored to the head
# ---------------------------------------------------------------------------

def _r3(v):
    """Round to the 3-decimal grid the gcode text uses; kill -0.0."""
    r = round(v, 3)
    return 0.0 if r == 0 else r


def strokes_bounds(strokes):
    b = None
    for st in strokes:
        for p in st.points:
            if b is None:
                b = [p[0], p[1], p[0], p[1]]
            else:
                b[0] = min(b[0], p[0])
                b[1] = min(b[1], p[1])
                b[2] = max(b[2], p[0])
                b[3] = max(b[3], p[1])
    return b


def anchor_reference(bounds, anchor_x, anchor_y):
    """The design point that sits on the head, per the anchor grid.

    Named in design (screen) terms: left/right are the design's min/max X;
    bottom is the design's bottom edge, top its top edge. The loader's
    Y flip is already resolved at this point (min Y in the internal frame
    is the design's bottom edge, max Y its top edge), so each axis is a
    plain min/centre/max pick.
    """
    rx = {"left": bounds[0], "center": (bounds[0] + bounds[2]) / 2.0,
          "right": bounds[2]}[anchor_x]
    ry = {"bottom": bounds[1], "center": (bounds[1] + bounds[3]) / 2.0,
          "top": bounds[3]}[anchor_y]
    return (rx, ry)


def render_gcode_current(strokes, stats, meta):
    """Current-position start mode: G91 relative moves in a frame whose
    origin is wherever the head sits when the job starts. The strokes
    have already been translated so the anchor point is the frame origin.

    Deltas are computed between successive rounded frame positions, so
    they telescope exactly on the file's 0.001 mm grid: the closing
    return move (after S0/M5, mirroring the absolute footer) brings the
    head back to the start point with net zero displacement. Zero-length
    deltas are not emitted at all, so no move in the file is a no-op and
    nothing assumes a machine or workspace origin.
    """
    b = stats["bounds"]
    lines = [
        "; LaserForge %s -- SVG -> GRBL gcode" % VERSION,
        "; Source: %s" % meta["source"],
        "; Mode: %s" % meta["mode"],
        "; Start mode: current position",
        "; Anchor: %s / %s (those design edges sit on the head)"
        % (meta["anchor_x"], meta["anchor_y"]),
        "; Bounds (relative to start): X%s Y%s to X%s Y%s"
        % (fmt(b[0]), fmt(b[1]), fmt(b[2]), fmt(b[3])),
        "; Burn distance: %.1f mm" % stats["burn"],
        "; Travel distance: %.1f mm" % stats["travel"],
        "; Estimated time: %.2f min" % stats["minutes"],
        "; Layers: %s" % ", ".join(meta["layer_names"]),
        "G21",
        "G91",
        "M4",
        "S0",
    ]
    lines[3:3] = _rotary_header_lines(meta)
    pos = (0.0, 0.0)

    def move(code, target, extra=""):
        nonlocal pos
        t = (_r3(target[0]), _r3(target[1]))
        d = (t[0] - pos[0], t[1] - pos[1])
        if d == (0.0, 0.0):
            return False
        lines.append("%s X%s Y%s%s" % (code, fmt(d[0]), fmt(d[1]), extra))
        pos = t
        return True

    group = None
    for st in strokes:
        key = (st.layer, st.op, st.pass_no)
        if key != group:
            group = key
            lines.append("; Layer '%s' -- %s, %s mm/min, %g%% (S%d), pass %d/%d"
                         % (st.layer, st.op, fmt_feed(st.feed), st.power, st.s,
                            st.pass_no, st.passes))
        move("G0", st.points[0])
        first = True
        for p in st.points[1:]:
            extra = " F%s S%d" % (fmt_feed(st.feed), st.s) if first else ""
            if move("G1", p, extra):
                first = False
        lines.append("S0")
    lines += ["; End of job", "S0", "M5"]
    move("G0", (0.0, 0.0))
    return "\n".join(lines) + "\n"


def verify_gcode_current(text, bed_w, bed_h):
    """Re-read current-position output: integrate the G91 deltas from the
    frame origin, recompute the burn bounds, check the design's span fits
    within the bed size as a maximum span (there is no fixed bed position
    to check against -- the job runs wherever the head is parked), and
    confirm the head ends back at the start point."""
    x = y = 0.0
    s = 0
    bounds = None
    problems = []
    saw_g91 = False

    def grow(p):
        nonlocal bounds
        if bounds is None:
            bounds = [p[0], p[1], p[0], p[1]]
        else:
            bounds[0] = min(bounds[0], p[0])
            bounds[1] = min(bounds[1], p[1])
            bounds[2] = max(bounds[2], p[0])
            bounds[3] = max(bounds[3], p[1])

    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith(";"):
            continue
        cmd = None
        dx = dy = 0.0
        ns = s
        for tok in line.split():
            head = tok[0]
            if head == "G":
                cmd = tok
                if tok == "G91":
                    saw_g91 = True
                elif tok == "G90":
                    problems.append("G90 found in current-position output")
            elif head == "X":
                dx = float(tok[1:])
            elif head == "Y":
                dy = float(tok[1:])
            elif head == "S":
                ns = int(float(tok[1:]))
        if cmd in ("G0", "G1"):
            ox, oy = x, y
            x, y = x + dx, y + dy
            if cmd == "G1" and ns > 0:
                grow((ox, oy))
                grow((x, y))
        s = ns
    if not saw_g91:
        problems.append("current-position output has no G91")
    if bounds is not None:
        w = bounds[2] - bounds[0]
        h = bounds[3] - bounds[1]
        if w > bed_w + 0.001 or h > bed_h + 0.001:
            problems.append("design span %.3f x %.3f mm exceeds the bed "
                            "size (%.0f x %.0f mm) as a maximum span"
                            % (w, h, bed_w, bed_h))
    if abs(x) > 0.0015 or abs(y) > 0.0015:
        problems.append("head does not return to the start point "
                        "(net displacement X%s Y%s)" % (fmt(x), fmt(y)))
    return bounds, problems


# ---------------------------------------------------------------------------
# Machine profiles (piece 2): mm_per_rotation is a machine+chuck constant.
# It lives in a JSON profile store -- never in a job file or on a cup --
# because it does not change when the cup does. The store is created on
# first use, seeded with Richard's Falcon Pro values.
# ---------------------------------------------------------------------------

SEED_PROFILES = {
    "falcon-pro": {
        "name": "Falcon Pro",
        "machine": "CR-Laser Falcon Pro 10W (GRBL, chuck rotary)",
        "mm_per_rotation": 44.5,
        "status": "measured 2026-11-21; confirmation proof burn pending",
        "notes": ("Measured via the LightBurn route on 2026-11-21: the "
                  "Williams logo's intended wrap was 2.5 in and it burned "
                  "4.125 in at mm_per_rotation 73.4 (a derived value); "
                  "true = 73.4 x 2.5 / 4.125 = 44.48, set to 44.5. One "
                  "constant for the machine+chuck; per cup, only the "
                  "circumference changes."),
    },
}


def load_profiles(path):
    """Load the profile store, creating it (seeded) on first use."""
    if not os.path.exists(path):
        store = {"profiles": {k: dict(v) for k, v in SEED_PROFILES.items()},
                 "calibration_pending": None}
        save_profiles(path, store)
        return store
    try:
        with open(path, "r") as fh:
            store = json.load(fh)
    except (OSError, ValueError) as exc:
        die("cannot read profiles file %s: %s" % (path, exc))
    if not isinstance(store, dict) or not isinstance(store.get("profiles"), dict):
        die("profiles file %s: expected an object with a 'profiles' object"
            % path)
    store.setdefault("calibration_pending", None)
    return store


def save_profiles(path, store):
    try:
        with open(path, "w") as fh:
            json.dump(store, fh, indent=2)
            fh.write("\n")
    except OSError as exc:
        die("cannot write profiles file %s: %s" % (path, exc))


def get_profile(store, key, path):
    prof = store["profiles"].get(key)
    if prof is None:
        die("profile %r not found in %s (have: %s)"
            % (key, path, ", ".join(sorted(store["profiles"])) or "none"))
    if prof.get("mm_per_rotation") is None:
        die("profile %r in %s has no mm_per_rotation -- calibrate it first "
            "(--calibrate-generate / --calibrate-apply)" % (key, path))
    return prof


def resolve_circumference(cands):
    """cands: list of (label, mm value) with Nones already removed.
    The circumference may be given exactly one way."""
    if len(cands) > 1:
        die("give the circumference only one way (got %s)"
            % " and ".join(label for label, _ in cands))
    return cands[0][1] if cands else None


def circumference_candidates(mm=None, inch=None, diameter_in=None):
    cands = []
    if mm is not None:
        cands.append(("--circumference-mm", float(mm)))
    if inch is not None:
        cands.append(("--circumference-in", float(inch) * IN_MM))
    if diameter_in is not None:
        cands.append(("--diameter-in", float(diameter_in) * IN_MM * math.pi))
    return cands


def show_profile(args):
    path = args.profiles or DEFAULT_PROFILES_PATH
    key = args.profile or "falcon-pro"
    store = load_profiles(path)
    prof = get_profile(store, key, path)
    print("profile '%s' (%s)" % (key, path))
    print("  name: %s" % prof.get("name", key))
    if prof.get("machine"):
        print("  machine: %s" % prof["machine"])
    print("  mm_per_rotation: %s mm" % fmt(float(prof["mm_per_rotation"])))
    print("  status: %s" % prof.get("status", ""))
    if prof.get("notes"):
        print("  notes: %s" % prof["notes"])
    pend = store.get("calibration_pending")
    if pend:
        print("  calibration pending: test arc %s mm at mm_per_rotation %s "
              "on circumference %s mm -- measure the burned arc and run "
              "--calibrate-apply"
              % (fmt(pend["test_arc_mm"]), fmt(pend["mm_per_rotation_used"]),
                 fmt(pend["circumference_mm"])))
    return 0


def calibrate_generate(args):
    """Write the calibration burn file: one straight line whose intended
    wrap arc is the test length, emitted with the profile's CURRENT
    mm_per_rotation, current-position start. Also records a pending
    calibration so --calibrate-apply needs only the measured value."""
    path = args.profiles or DEFAULT_PROFILES_PATH
    key = args.profile or "falcon-pro"
    store = load_profiles(path)
    prof = get_profile(store, key, path)
    circ = resolve_circumference(circumference_candidates(
        args.circumference_mm, args.circumference_in, args.diameter_in))
    if circ is None:
        die("--calibrate-generate needs the cup circumference: "
            "--circumference-mm, --circumference-in or --diameter-in")
    if circ <= 0:
        die("circumference must be > 0")
    if args.test_arc_mm is not None and args.test_arc_in is not None:
        die("give the test arc only one way (--test-arc-in or --test-arc-mm)")
    if args.test_arc_mm is not None:
        arc_mm = float(args.test_arc_mm)
    elif args.test_arc_in is not None:
        arc_mm = float(args.test_arc_in) * IN_MM
    else:
        arc_mm = 2.5 * IN_MM
    if arc_mm <= 0:
        die("test arc must be > 0")
    mpr = float(prof["mm_per_rotation"])
    factor = mpr / circ
    stroke = Stroke([(0.0, 0.0), (0.0, arc_mm * factor)],
                    float(args.speed), int(round(float(args.power) * 10.0)),
                    float(args.power), "calibration", "line", 1, 1)
    strokes = [stroke]
    stats = analyze(strokes)
    meta = {"source": "calibration line (generated; no input SVG)",
            "mode": "rotary", "anchor_x": "left", "anchor_y": "bottom",
            "layer_names": ["calibration"],
            "rotary": {"profile_key": key,
                       "profile_name": prof.get("name", key),
                       "mm_per_rotation": mpr,
                       "status": prof.get("status", ""),
                       "circumference_mm": circ, "factor": factor},
            "calibration": {"test_arc_mm": arc_mm}}
    text = render_gcode_current(strokes, stats, meta)
    bounds, problems = verify_gcode_current(text, DEFAULT_BED[0], DEFAULT_BED[1])
    if bounds is not None:
        emitted = bounds[3] - bounds[1]
        if abs(emitted - arc_mm * factor) > 0.0025:
            problems.append("emitted wrap travel %.3f mm does not match "
                            "intended arc x wrap scale (%.3f mm)"
                            % (emitted, arc_mm * factor))
    if problems:
        for p in problems[:10]:
            print("verify: %s" % p, file=sys.stderr)
        die("verification failed -- output NOT written (%d problem(s))"
            % len(problems), code=3)
    out_path = args.output or "calibration.gcode"
    with open(out_path, "w") as fh:
        fh.write(text)
    store["calibration_pending"] = {
        "profile": key,
        "circumference_mm": circ,
        "test_arc_mm": arc_mm,
        "mm_per_rotation_used": mpr,
        "generated": datetime.datetime.now().isoformat(timespec="seconds"),
        "output": out_path,
    }
    save_profiles(path, store)
    print("calibration file for profile '%s' (mm_per_rotation %s mm)"
          % (key, fmt(mpr)))
    print("circumference: %s mm (%s in); intended test arc: %s mm (%s in)"
          % (fmt(circ), fmt(circ / IN_MM), fmt(arc_mm), fmt(arc_mm / IN_MM)))
    print("emitted wrap travel: %s mm (arc x mm_per_rotation / circumference)"
          % fmt(arc_mm * factor))
    print("wrote: %s" % out_path)
    print("next: burn it on tape, measure the burned arc, then run "
          "--calibrate-apply --measured-in <your measurement>")
    return 0


def calibrate_apply(args):
    """Finish a calibration: true = used x intended / measured, written
    back into the profile the generate step recorded."""
    path = args.profiles or DEFAULT_PROFILES_PATH
    store = load_profiles(path)
    pend = store.get("calibration_pending")
    if not pend:
        die("no calibration is pending -- run --calibrate-generate first, "
            "burn the line on tape, measure the arc, then apply")
    if args.measured_mm is not None and args.measured_in is not None:
        die("give the measurement only one way (--measured-in or "
            "--measured-mm)")
    if args.measured_in is not None:
        measured = float(args.measured_in) * IN_MM
    elif args.measured_mm is not None:
        measured = float(args.measured_mm)
    else:
        die("--calibrate-apply needs the measured arc: --measured-in or "
            "--measured-mm")
    if measured <= 0:
        die("measured arc must be > 0")
    key = pend["profile"]
    prof = get_profile(store, key, path)
    used = float(pend["mm_per_rotation_used"])
    intended = float(pend["test_arc_mm"])
    new_exact = used * intended / measured
    new_value = round(new_exact, 2)
    old_value = float(prof["mm_per_rotation"])
    today = datetime.date.today().isoformat()
    prof["mm_per_rotation"] = new_value
    prof["status"] = "measured %s" % today
    note = ("Calibrated %s: test arc %s mm intended, measured %s mm at "
            "mm_per_rotation %s -> true = %s x %s / %s = %.4f, stored %s."
            % (today, fmt(intended), fmt(measured), fmt(used), fmt(used),
               fmt(intended), fmt(measured), new_exact, fmt(new_value)))
    prof["notes"] = ((prof.get("notes") or "") + " " + note).strip()
    store["calibration_pending"] = None
    save_profiles(path, store)
    print("calibration applied to profile '%s':" % key)
    print("  mm_per_rotation = used x intended / measured")
    print("                  = %s x %s / %s" % (fmt(used), fmt(intended),
                                                fmt(measured)))
    print("                  = %.4f -> %s mm (was %s mm)"
          % (new_exact, fmt(new_value), fmt(old_value)))
    print("  status: %s" % prof["status"])
    return 0


# ---------------------------------------------------------------------------
# CLI / job assembly
# ---------------------------------------------------------------------------

def parse_bed(text):
    m = re.match(r"^\s*([\d.]+)\s*x\s*([\d.]+)\s*$", text or "")
    if not m:
        die("--bed must look like 400x415 (got %r)" % text)
    return (float(m.group(1)), float(m.group(2)))


def _anchor_pair(name):
    pairs = {"center": ("center", "center"), "min": ("left", "bottom"),
             "max": ("right", "top")}
    if name not in pairs:
        die("anchor must be 'center', 'min' or 'max' (got %r)" % (name,))
    return pairs[name]


def load_job(args):
    job = {"mode": "flat", "offset": (0.0, 0.0), "bed": DEFAULT_BED,
           "start_mode": "current", "anchor_x": "left",
           "anchor_y": "bottom", "layers": [],
           "profile": "falcon-pro", "circumference_mm": None}
    if args.job:
        try:
            with open(args.job, "r") as fh:
                raw = json.load(fh)
        except (OSError, ValueError) as exc:
            die("cannot read job file %s: %s" % (args.job, exc))
        if "mode" in raw:
            job["mode"] = raw["mode"]
        if "offset" in raw:
            job["offset"] = (float(raw["offset"].get("x", 0.0)),
                             float(raw["offset"].get("y", 0.0)))
        if "bed" in raw:
            job["bed"] = (float(raw["bed"].get("width", DEFAULT_BED[0])),
                          float(raw["bed"].get("height", DEFAULT_BED[1])))
        start_mode = raw.get("start_mode", raw.get("start_from"))
        if start_mode is not None:
            job["start_mode"] = start_mode
        if "anchor" in raw:
            job["anchor_x"], job["anchor_y"] = _anchor_pair(raw["anchor"])
        if "anchor_x" in raw:
            job["anchor_x"] = raw["anchor_x"]
        if "anchor_y" in raw:
            job["anchor_y"] = raw["anchor_y"]
        if "profile" in raw:
            job["profile"] = raw["profile"]
        job_cands = circumference_candidates(
            raw.get("circumference_mm"), raw.get("circumference_in"),
            raw.get("diameter_in"))
        if job_cands:
            job["circumference_mm"] = resolve_circumference(job_cands)
        if "layers" not in raw or not raw["layers"]:
            die("job file %s: no layers defined" % args.job)
        job["layers"] = [normalize_layer(l, i) for i, l in enumerate(raw["layers"])]
    else:
        sel = None
        if args.select:
            sel = []
            for tok in args.select.split(","):
                tok = tok.strip()
                if not tok:
                    continue
                sel.append(int(tok) if re.fullmatch(r"\d+", tok) else tok)
        layer = {"name": "layer-1", "select": sel,
                 "ops": [{"op": args.op, "speed": args.speed,
                          "power": args.power, "passes": args.passes,
                          "spacing": args.spacing, "angle": args.angle,
                          "outline": args.outline}]}
        job["layers"] = [normalize_layer(layer, 0)]
    # CLI overrides
    if args.mode:
        job["mode"] = args.mode
    if args.offset_x is not None or args.offset_y is not None:
        ox, oy = job["offset"]
        job["offset"] = (args.offset_x if args.offset_x is not None else ox,
                         args.offset_y if args.offset_y is not None else oy)
    if args.bed:
        job["bed"] = parse_bed(args.bed)
    if args.start_mode:
        job["start_mode"] = args.start_mode
    if args.anchor:
        job["anchor_x"], job["anchor_y"] = _anchor_pair(args.anchor)
    if args.anchor_x:
        job["anchor_x"] = args.anchor_x
    if args.anchor_y:
        job["anchor_y"] = args.anchor_y
    if args.profile:
        job["profile"] = args.profile
    cli_cands = circumference_candidates(
        args.circumference_mm, args.circumference_in, args.diameter_in)
    if cli_cands:
        job["circumference_mm"] = resolve_circumference(cli_cands)
    if job["start_mode"] not in ("current", "absolute"):
        die("start_mode must be 'current' or 'absolute' (got %r)"
            % (job["start_mode"],))
    if job["anchor_x"] not in ("left", "center", "right"):
        die("anchor_x must be 'left', 'center' or 'right' (got %r)"
            % (job["anchor_x"],))
    if job["anchor_y"] not in ("top", "center", "bottom"):
        die("anchor_y must be 'top', 'center' or 'bottom' (got %r)"
            % (job["anchor_y"],))
    return job


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="laserforge",
        description="LaserForge -- SVG -> GRBL gcode (flat + rotary).")
    ap.add_argument("input", nargs="?", default=None,
                    help="input SVG file (not needed for --show-profile or "
                         "the --calibrate-* steps)")
    ap.add_argument("-o", "--output",
                    help="output gcode file (default: input name with .gcode)")
    ap.add_argument("--job", help="JSON job file (layers/mode/offset/bed)")
    ap.add_argument("--mode", default=None,
                    help="'flat' (default) or 'rotary'")
    ap.add_argument("--profile", default=None,
                    help="machine profile key for rotary (default: "
                         "falcon-pro)")
    ap.add_argument("--profiles", default=None,
                    help="profiles JSON store (default: profiles.json "
                         "next to laserforge.py)")
    ap.add_argument("--show-profile", action="store_true",
                    help="print the selected machine profile and exit")
    ap.add_argument("--circumference-mm", type=float, default=None,
                    help="rotary: cup circumference in mm")
    ap.add_argument("--circumference-in", type=float, default=None,
                    help="rotary: cup circumference in inches")
    ap.add_argument("--diameter-in", type=float, default=None,
                    help="rotary: cup diameter in inches (circumference "
                         "= pi x diameter)")
    ap.add_argument("--calibrate-generate", action="store_true",
                    help="write a calibration burn file (one line whose "
                         "intended wrap arc is the test length) using the "
                         "profile's current mm_per_rotation, and exit")
    ap.add_argument("--calibrate-apply", action="store_true",
                    help="finish a calibration from the measured arc "
                         "(--measured-in / --measured-mm) and exit")
    ap.add_argument("--test-arc-in", type=float, default=None,
                    help="calibration test arc in inches (default: 2.5)")
    ap.add_argument("--test-arc-mm", type=float, default=None,
                    help="calibration test arc in mm")
    ap.add_argument("--measured-in", type=float, default=None,
                    help="calibration: the burned arc you measured, inches")
    ap.add_argument("--measured-mm", type=float, default=None,
                    help="calibration: the burned arc you measured, mm")
    ap.add_argument("--op", default="line", choices=["line", "fill", "cut"],
                    help="single-layer op when no --job is given (default: line)")
    ap.add_argument("--speed", type=float, default=1000.0, help="mm/min")
    ap.add_argument("--power", type=float, default=50.0, help="percent 0-100")
    ap.add_argument("--passes", type=int, default=1)
    ap.add_argument("--spacing", type=float, default=0.1, help="fill hatch spacing, mm")
    ap.add_argument("--angle", type=float, default=0.0, help="fill hatch angle, degrees")
    ap.add_argument("--outline", action="store_true",
                    help="fill: add an outline pass around filled shapes")
    ap.add_argument("--select", default=None,
                    help="comma list of shape ids/labels or document indexes")
    ap.add_argument("--start-mode", default=None,
                    choices=["current", "absolute"],
                    help="start mode: 'current' (default) anchors the job "
                         "to wherever the head sits; 'absolute' uses "
                         "workspace coordinates on the bed")
    ap.add_argument("--anchor", default=None,
                    choices=["center", "min", "max"],
                    help="current mode, combined anchor: 'center' = design "
                         "centre on the head, 'min' = left/bottom edges "
                         "(the default), 'max' = right/top edges")
    ap.add_argument("--anchor-x", default=None,
                    choices=["left", "center", "right"],
                    help="current mode: which design X edge sits on the head")
    ap.add_argument("--anchor-y", default=None,
                    choices=["top", "center", "bottom"],
                    help="current mode: which design Y edge sits on the "
                         "head (design/screen terms: 'top' = the design's "
                         "top edge)")
    ap.add_argument("--offset-x", type=float, default=None)
    ap.add_argument("--offset-y", type=float, default=None)
    ap.add_argument("--bed", default=None, help="bed size, e.g. 400x415")
    args = ap.parse_args(argv)

    # Profile / calibration steps need no input SVG and no job.
    if args.show_profile:
        return show_profile(args)
    if args.calibrate_apply:
        return calibrate_apply(args)
    if args.calibrate_generate:
        return calibrate_generate(args)
    if not args.input:
        die("an input SVG is required (or use --show-profile / "
            "--calibrate-generate / --calibrate-apply)")

    job = load_job(args)

    if job["mode"] not in ("flat", "rotary"):
        die("mode must be 'flat' or 'rotary' (got %r)" % (job["mode"],))
    rotary = None
    if job["mode"] == "rotary":
        if job["circumference_mm"] is None:
            die("mode 'rotary' needs the cup circumference: "
                "--circumference-mm, --circumference-in or --diameter-in "
                "(or the matching job-file keys)")
        if job["circumference_mm"] <= 0:
            die("circumference must be > 0")
        profiles_path = args.profiles or DEFAULT_PROFILES_PATH
        store = load_profiles(profiles_path)
        prof = get_profile(store, job["profile"], profiles_path)
        mpr = float(prof["mm_per_rotation"])
        rotary = {"profile_key": job["profile"],
                  "profile_name": prof.get("name", job["profile"]),
                  "mm_per_rotation": mpr,
                  "status": prof.get("status", ""),
                  "circumference_mm": job["circumference_mm"],
                  "factor": mpr / job["circumference_mm"]}
    shapes, report = load_svg(args.input)
    if not shapes:
        die("%s: no usable geometry found" % args.input)
    shapes = cleanup_shapes(shapes, report)
    for line in report:
        print(line)

    ox, oy = job["offset"]
    if ox or oy:
        for sh in shapes:
            sh.points = [(x + ox, y + oy) for (x, y) in sh.points]

    layer_report = []
    strokes = build_strokes(job, shapes, layer_report)
    for line in layer_report:
        print(line)
    if not strokes:
        die("nothing to burn: the job produced no toolpaths")

    if job["start_mode"] == "current":
        # Translate the whole job so the anchor point is the frame
        # origin: the emitted file then lives entirely relative to
        # wherever the head is parked when the job starts.
        db = strokes_bounds(strokes)
        rx, ry = anchor_reference(db, job["anchor_x"], job["anchor_y"])
        for st in strokes:
            st.points = [(x - rx, y - ry) for (x, y) in st.points]

    design_y_span = None
    if rotary is not None:
        # Rotary: the design's Y is the wrap (circumference) direction.
        # Scale it by mm_per_rotation / circumference so the emitted Y
        # travel is the chuck travel that lays the design at true size
        # on the cup; X (along the cup) is unscaled. The anchor was
        # resolved in design terms above, before this scaling.
        sb = strokes_bounds(strokes)
        design_y_span = sb[3] - sb[1]
        f = rotary["factor"]
        for st in strokes:
            st.points = [(x, y * f) for (x, y) in st.points]

    stats = analyze(strokes)
    meta = {"source": os.path.basename(args.input), "mode": job["mode"],
            "start_mode": job["start_mode"], "anchor_x": job["anchor_x"],
            "anchor_y": job["anchor_y"],
            "layer_names": [l["name"] for l in job["layers"] if l["output"]],
            "rotary": rotary}
    if job["start_mode"] == "current":
        text = render_gcode_current(strokes, stats, meta)
    else:
        text = render_gcode(strokes, stats, meta)

    bed_w, bed_h = job["bed"]
    if job["start_mode"] == "current":
        bounds, problems = verify_gcode_current(text, bed_w, bed_h)
    else:
        bounds, problems = verify_gcode(text, bed_w, bed_h)
    if rotary is not None and bounds is not None:
        # Rotary addition: the emitted wrap (Y) travel integrated back
        # from the file must equal the scaled design span -- design
        # span x mm_per_rotation / circumference -- within rounding.
        expected_wrap = stats["bounds"][3] - stats["bounds"][1]
        emitted_wrap = bounds[3] - bounds[1]
        if abs(emitted_wrap - expected_wrap) > 0.0025:
            problems.append(
                "rotary wrap travel mismatch: emitted Y extent %.3f mm, "
                "expected %.3f mm (design wrap span %.3f mm x "
                "mm_per_rotation %.3f / circumference %.3f)"
                % (emitted_wrap, expected_wrap, design_y_span,
                   rotary["mm_per_rotation"], rotary["circumference_mm"]))
    if problems:
        for p in problems[:10]:
            print("verify: %s" % p, file=sys.stderr)
        die("verification failed -- output NOT written (%d problem(s))"
            % len(problems), code=3)
    if bounds is None:
        die("verification failed -- no burn moves found in generated output",
            code=3)

    out_path = args.output or os.path.splitext(args.input)[0] + ".gcode"
    with open(out_path, "w") as fh:
        fh.write(text)

    if job["start_mode"] == "current":
        print("bounds (relative to start point): X %s..%s  Y %s..%s"
              % (fmt(bounds[0]), fmt(bounds[2]), fmt(bounds[1]),
                 fmt(bounds[3])))
    else:
        print("bounds (design): X %s..%s  Y %s..%s"
              % (fmt(bounds[0]), fmt(bounds[2]), fmt(bounds[1]),
                 fmt(bounds[3])))
    print("burn distance: %.1f mm" % stats["burn"])
    print("travel distance: %.1f mm" % stats["travel"])
    print("estimated time: %.2f min (%.1f s)  "
          "[estimate: sum(length/feed); rapids at %d mm/min]"
          % (stats["minutes"], stats["minutes"] * 60.0, RAPID_FEED))
    if job["start_mode"] == "current":
        print("emitted bounds: X %s..%s  Y %s..%s (span %.3f x %.3f mm) -- "
              "span within bed max %g x %g mm; head returns to the start "
              "point"
              % (fmt(bounds[0]), fmt(bounds[2]), fmt(bounds[1]),
                 fmt(bounds[3]), bounds[2] - bounds[0],
                 bounds[3] - bounds[1], bed_w, bed_h))
    else:
        print("emitted bounds: X %s..%s  Y %s..%s (%.3f x %.3f mm) -- "
              "inside bed %g x %g mm"
              % (fmt(bounds[0]), fmt(bounds[2]), fmt(bounds[1]),
                 fmt(bounds[3]), bounds[2] - bounds[0],
                 bounds[3] - bounds[1], bed_w, bed_h))
    if rotary is not None:
        print("rotary: profile '%s', mm per rotation %s mm, circumference "
              "%s mm (%s in), wrap scale Y x %.6f"
              % (rotary["profile_key"], fmt(rotary["mm_per_rotation"]),
                 fmt(rotary["circumference_mm"]),
                 fmt(rotary["circumference_mm"] / IN_MM), rotary["factor"]))
        print("emitted wrap travel: Y extent %s mm (design wrap span %s mm "
              "x %.6f)"
              % (fmt(bounds[3] - bounds[1]), fmt(design_y_span),
                 rotary["factor"]))
    print("wrote: %s" % out_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
