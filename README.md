# fixture-check

A small Python package that automatically verifies checking and locating fixtures. Built on top of
[cad-khana](https://github.com/cyberchitta/cad-khana): you declare the role of each body in the fixture (part, datum
pads, locating pins, clamps, contour blocks, gauge), and the package builds both cad-khana assertions and
fixture-specific checks. The output is a cad-khana `Assembly`, so `khana check`, `khana show`, and `khana diff` work
out of the box.

The goal is to catch functional errors in fixtures designed by a human or an AI agent that slip past "valid solid, no
interference": the part cannot be loaded, a diamond pin is rotated 90°, a clamp presses unsupported sheet metal, a pin
does not fit the hole in the worst case, or a block is floating in mid-air.

## Installation

cad-khana requires Python 3.13. cad-khana is pinned to a tested commit in `pyproject.toml`.

```
git clone https://github.com/DailyDana/fikstur-kontrol
cd fikstur-kontrol
uv venv --python 3.13
uv pip install -e . pytest
```

If cad-khana already lives in its own environment, install into that one without dependencies:

```
uv pip install --python <path-to-cad-khana-venv-python> --no-deps -e .
```

## Usage

```python
from fixture_check import Fixture, Tolerance

f = (Fixture.step("fixture_assembly.step", part="Sheet_Bracket")
     .base("Base_Plate")
     .datum_a(["Pad_A1", "Pad_A2", "Pad_A3"])
     .round_pin("Pin_B", hole=Tolerance.deviation(9, 0, 0.10), pin=Tolerance.iso(9, "e7"))
     .diamond_pin("Pin_C", hole=Tolerance.deviation(9, 0, 0.10), pin=Tolerance.iso(9, "e7"))
     .clamp("Clamp_Left_Pad", arm=["Clamp_Left_Arm", "Clamp_Left_Handle"],
            pivot=((-42, -66, 102), (1, 0, 0)), opening=100)
     .contour("Block_Left", gap=3.0)
     .flush("Block_Left", direction=(0, 1, 0))
     .removal(direction=(0, 0, 1), distance=65)
     .go_nogo_gauge("GO_NOGO", direction=(0, -1, 0), retract=10, stroke=10, go_dia=7.5, nogo_dia=8.1,
                    hole=Tolerance.plus_minus(8, 0.10), position_tol=0.40, bushing="Guide_Bushing"))
assembly = f.assembly()
```

```
khana check fixture_spec.py
khana show outputs/fixture_spec/mechanism.json --failed
```

Full example: [examples/bracket_fixture.py](examples/bracket_fixture.py) (sheet-metal bracket checking fixture and
seven intentional fault variants).

When the input is a STEP file, every body must have a name. You can also pass build123d shapes via
`Fixture.from_bodies({"name": shape, ...}, part=...)`. `Fixture` is immutable; `add_body` and `replace_body` return a
new specification (useful for deriving faulty variants).

## Checks

Assertion names in the report start with these IDs. Assertions starting with `warn:` are unbounded records: their value
is reported without turning the run red.

| ID | What it checks | How |
|---|---|---|
| `solid_count`, `no_interference` | Part is a single solid; part interferes with no body; fixture bodies do not interfere with each other | cad-khana |
| `known_interference` | Whether an accepted overlap is still present (warns if it disappears) | `known_interference(a, b, reason)` |
| `contact`, `clear` | Part touches only datum pads and clamp pads; stays at least `min_clearance` (0.1 mm) away from every other body | cad-khana tangent contact and distance |
| `F1` | Exactly 3 pads in contact (or 3+ with `n_2_1=True`), coplanar pads, shortest altitude of support triangle, pad seating ratio | Contact patches extracted from geometry |
| `F2` (warn) | Whether the center of mass projected along gravity falls inside the support polygon, and margin to the edge in mm | `gravity=`, enforced with `strict_stability=True` |
| `F3` | Whether each clamp's pressure center lies inside a pad's outer boundary, presses opposite to the pad, and has solid part material in between; unclamped pad count (warn) | Contact centroid projected onto pad face |
| `pin:` | Pin cylindrical section spans full part thickness, CAD clearance, shoulder gap (including tapered shoulder), CAD diameter in tolerance zone, worst-case clearance (ISO 286) and with CAD axis offset deducted, roundness of round pin, datum shift | Pin cross-section in part thickness band |
| `pin:...diamond_*`, `slot_orientation` | Diamond pin relief along B→C and contacting direction perpendicular to B→C; part slot parallel to B→C; fails if a cylindrical pin is declared as diamond | Cross-section widths, slot end half-cylinders |
| `F6` | Constrained degrees of freedom = 6 (3 pads + round 2 + diamond 1) and pin layout is one round + one diamond; pin type is also verified geometrically (roundness, diamond relief) | Count + cross-section |
| `gap`, `flush` | Contour gap within tolerance; block face coplanar with part face in the given direction (same-facing normals; opposite-facing with `opposed=True`) | Distance and plane offset |
| `K8` | Every fixture body is connected to the base by contact (or a guided fit within `connection_tol`) | Contact graph |
| `removal_path` | Part can be removed along the given direction without obstruction | cad-khana `swept` (exact sweep) |
| `K9` | Clamp arm (pad + `arm` bodies) rotates `opening` degrees around `pivot` without colliding with the part or fixed bodies | Rotated in `step` (1°) increments checking volume interference; sampled |
| `gauge:` | GO/NO-GO gauge path is clear including part and bushing, part has a coaxial hole that GO passes completely through, GO = virtual condition, NO-GO = LMC (at MMC), bushing is coaxial with sufficient L/D; anything missing fails | Sweep and cylinder measurements |

## Limitations

- Nominal geometry is checked. Tolerances are accounted for only in pin-hole fits and gauge diameters; there is no
  tolerance stack-up.
- Gauge motion is checked only as a straight stroke. Clamp opening (K9) is sampled: a thin obstacle between two steps
  could be missed; reduce `step` if needed. Pass only the moving parts (arm, pad, handle) to `clamp(group=...)`; if a
  fixed base matches the pattern, the removal path check will ignore it.
- Gap uniformity reads high around rounded corners, so it is a warning by default.
- No metrology/operator access, manufacturability, or force analysis.
- cad-khana is an early-stage project; its API may change.

## Tests

```
python -m pytest -q -p no:warnings
```

Passing and failing cases for every check on a synthetic fixture, ISO 286 values, and the seven faulty variants of the
real bracket fixture (`examples/bracket_fixture_assembly.step`).

## License

GNU Affero General Public License v3.0 only (AGPL-3.0-only). See [LICENSE](LICENSE). If you distribute a modified
version, or let people use one over a network, you must make its source available under the same license.
