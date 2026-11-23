# LaserForge

**SVG in → GRBL gcode out.** A small, honest tool for diode laser engravers,
built for the CR-Laser Falcon Pro and its memory-card workflow: design,
export the `.gcode` file, put it on the card, burn.

Free and open source under the [MIT License](LICENSE).
© 2026 Richard Seals (Un4given AI and Gaming Development).

## Get the app

**Just want to use it? Download [`laserforge.html`](laserforge.html) — that's
the whole app.** One file, no install, no internet, no account. Your designs
and settings never leave your device.

- **Windows / Mac:** save the file and double-click it (it opens in your
  browser).
- **iPad / iPhone:** open the file in Safari, then Share → **Add to Home
  Screen** to keep it like an app.
- Works fully offline once saved.

## What it does

- Import an SVG (outlines/paths) and preview it on the bed
- Line, fill (with spacing/angle), and cut operations — power %, speed
  mm/min, passes
- **Flat work** with Current Position starting and a 9-point anchor grid
- **Rotary work** with machine profiles and a true mm-per-rotation model —
  the Falcon Pro profile ships with a constant *measured on real hardware*
  (44.5 mm/rotation, confirmed by a tape-test burn)
- **Built-in calibration:** LaserForge generates a test-line file, you burn
  it, measure the line, type the measurement in — it corrects the rotary
  constant for you
- Export gcode for the memory card

## Honest limits

- Card export only — a browser page cannot connect to the laser directly.
- Vector outlines only: no raster/photo dithering, no text (convert text to
  outlines in your design tool first).
- One machine profile family ships today (Falcon Pro); the profile format
  is plain JSON if you want to add your own.

## Files in this repo

| File | What it is |
|---|---|
| `laserforge.html` | The app — the download most people want |
| `laserforge.js` | The engine as a standalone JS module/CLI (Node) |
| `laserforge.py` | The original Python engine (reference implementation) |
| `profiles.json` | Machine profiles |
| `PARITY.md` | Proof the JS port matches the Python engine byte-for-byte (118/118 checks) |

## Why it exists

Built at a workbench, not a whiteboard: every rotary number in here was
proven on a real machine with a tape measure before it shipped. No fake
promises — this is how you set it up, and it works.
