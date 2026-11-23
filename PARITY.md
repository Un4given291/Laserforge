# LaserForge v1 — JS/Python parity report

**Verdict: the JavaScript port is byte-identical to the Python engine
on every observable the acceptance suite checks.**

- `node parity-test.js` → **118/118 checks passed** (full output:
  `parity-output.txt`)
- `node smoke-test.js` → **7/7 checks passed** — the built
  `laserforge.html`'s own inline scripts, driven end to end
  (full output: `smoke-output.txt`)

## Method

1. `make_golden.py` ran the **Python** engine (`../laserforge.py`)
   over the 24 cases in `cases.json` — the existing `tests/`
   fixtures, the `tests/` job files, the calibration sequence, both
   guard refusals, and two new fixtures (`fixtures/units_in.svg`,
   `fixtures/transforms.svg`) covering inch-sized SVGs and transform
   attributes, which the Python CLI suite did not exercise. For each
   case it captured exit code, stdout, stderr, the emitted gcode
   file, and (for calibration cases) the resulting profile store,
   into `golden/`. All runs used relative paths in a scratch
   directory so no absolute path leaks into the captures; scratch
   profile stores were seeded identically for both engines.
2. `parity-test.js` replays the same case list against
   `laserforge.js` (as a Node CLI subprocess) and compares:
   exit code (exact), stdout (exact), stderr (exact), gcode file
   existence and **bytes** (exact), and profile stores after
   calibration cases (parsed deep-compare; see note 2 below).

No float tolerance was needed anywhere: the JS port reproduces
Python's formatting exactly, including round-half-even in `%.3f`
output and number printing. The port also mirrors Python's quirks
rather than smoothing them over (e.g. single-digit arc flags in
path data, `S`/`T` reflection rules, the cleanup report wording,
deltas computed between successive *rounded* positions in
current-position mode).

## Coverage vs TEST_RESULTS.md (46 Python acceptance checks)

Every one of the 46 checks in `../TEST_RESULTS.md` is an assertion
over the CLI's emitted file, its stdout, its exit code, or the
profile store — and all of those are byte-identical here:

| Python checks | JS parity case(s) |
|---|---|
| 1, 6 — square, bounds, burn distance, time estimate | `t1_square_line` |
| 2 — fill hatch count/placement | `t2_square_fill` |
| 4 — curves (C/A/Q/S/T, ellipse) | `t4_curves` |
| 5 — cleanup: dedupe, auto-join, unclosed warning | `t5_messy` |
| guards — rotary without circumference (rc 2), out-of-bed refusal (rc 3, no file) | `guard_rotary_nocirc`, `guard_oob` |
| 7, 8, 8b, 9 — current-position default, job-file centre anchor, right/top anchor, G91/header/zero-move rules | `t7_current_defaults`, `t8_job_center`, `t8b_current_right_top` |
| multi-layer job file | `example_job` |
| 10, 11, 12 — rotary wrap scaling, reference wrap, full wrap = exactly mm_per_rotation | `t10_rotary`, `t11_rotary_wrap`, `t12_rotary_fullwrap` |
| 13 — calibration generate/apply arithmetic (73.4 × 63.5 / 104.775 → stored 44.48), apply-with-nothing-pending (rc 2) | `t13_cal_generate`, `t13_cal_apply`, `t13_cal_apply_none` |
| 14 — rotary current-position simulation + header | same emitted file as `t10_rotary` (byte-identical ⇒ identical simulation) |
| 15–21 era behaviours — rotary absolute, 45° fill + outline, profile display, Falcon-profile calibration generate, missing input (rc 2) | `t15_rotary_absolute`, `t16_fill45_outline`, `show_profile`, `t20_cal_generate_falcon`, `t21_no_input` |
| beyond the Python suite — inch/mm unit handling, transforms (rotate/matrix/skew/rounded rect), job-file diameter input | `t17_units_in`, `t18_transforms`, `t19_job_rotary_diameter` |

**Acceptance behaviours that do not hold in JS: none found.**

Side-effect parity was also checked: like Python, the JS CLI does
not create the profiles file on flat runs or on a rotary run that
is refused for a missing circumference, and seeds it on the first
run that actually resolves a profile. A freshly seeded JS store
file is byte-identical to Python's.

## App smoke test

`smoke-test.js` extracts the two inline `<script>` blocks from the
**built** `laserforge.html`, runs them in a VM against a minimal
DOM stub, loads `tests/square20.svg` through the page's own file
input, presses Generate with the page defaults, and compares the
result against the JS CLI — byte-identical. It then runs the page's
calibration flow (generate → enter 4.125 in → apply) and checks the
stored Falcon Pro constant becomes 26.97 mm/rotation
(44.5 × 63.5 / 104.775) in localStorage, with the pending entry
cleared.

## Known non-parity items (cosmetic only)

1. **Malformed command lines.** Python's argparse prints its own
   usage/error text (exit 2). The JS CLI prints a one-line
   `error: ...` and also exits 2. No golden case covers this; exit
   codes match, wording differs.
2. **Profile stores after calibration** are compared parsed, not
   byte-wise, for one reason only: the `calibration_pending`
   `generated` field is a wall-clock timestamp in both engines.
   Everything else in the store — including the stored
   mm_per_rotation — matches exactly.
3. **Malformed SVG input.** Both engines refuse with exit 2 and a
   message naming the problem, but the wording comes from each
   platform's XML parser (Python's ElementTree vs the port's
   built-in parser), so the text differs.
4. **Date-stamped lines.** Calibration apply stamps
   `measured <today>` in both engines; goldens were captured the
   same day the parity run passed, so that line will differ across
   days by design — in Python too.
5. **HTML app calibration log** prints the exact calibration result
   with JavaScript's `toFixed(4)` (round-half-up) where the CLI
   uses Python-style round-half-even. Display only: the stored
   constant comes from the engine's half-even rounding in both.

## Reproducing

```sh
cd ~/workspace/laser-forge/v1
python3 make_golden.py     # regenerate goldens from the Python engine
node parity-test.js        # 118 checks
node build-html.js         # rebuild laserforge.html from sources
node smoke-test.js         # 7 checks against the built file
```
