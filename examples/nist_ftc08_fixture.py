"""Checking fixture for the NIST FTC-08 test part, built in code and verified with fixture-check.

    khana check examples/nist_ftc08_fixture.py               # actual design
    khana check examples/nist_ftc08_fixture.py:<variant>     # intentional fault variants, below

The part is "PMI Fully-Toleranced Test Case 8" from the NIST MBE PMI Validation and Conformance Testing project
(https://www.nist.gov/el/systems-integration-division-73400/mbe-pmi-validation-and-conformance-testing).
NIST: "The test cases, CAD models, and STEP files can be used without any restrictions." The STEP file here is the
geometry-only AP203 file, unchanged.

The part is a machined tray, 311.6 x 222.7 x 48.3 mm, open underneath, with a 10.16 mm flange around its base. From
the NIST drawing (inches):
- Datum A: the flange face (Z = 0 in the STEP file), flatness .015.
- Datum B: Ø.238 +.005/-.001 corner hole, perpendicular to A within Ø.015 at MMC.
- Datum C: Ø.238 +.005/-.001 corner hole, position Ø.020 at MMC to A|B.
- 2X Ø.238 +.005/-.001 corner holes, position Ø.023 at MMC to A|B|C.
The drawing note asks for exactly this restraint: A against its simulator, B and C engaged by simulators, load
applied to hold A. B and C are referenced RFS (no MMC modifier), which strictly calls for expanding pins; this fixture
uses fixed pins and reports the resulting shift (0.19 mm diametral) as a warning. Which two corners are B and C is read from the drawing's isometric view; the fixture is checked
for the choice below (B and C on the same long edge, 285.75 mm apart).

Fixture concept: the part sits as modelled, datum A down, on three hardened pads under the side walls. A round pin
in B and a diamond pin in C come up from below through the flange. Three swing-arm clamps press the 5 mm wide flat
band on top of the flange directly above each pad. A GO/NO-GO pin slides up through a bushing into one of the 2X
holes. Loading is straight down onto the pins with clamps open and the gauge pulled down.
"""

from pathlib import Path

from build123d import Box, Cylinder, Pos, Rot, import_step

from fixture_check import Fixture, Tolerance

STEP = Path(__file__).resolve().with_name("nist_ftc_08_asme1_rc.stp")
PART = "FTC08_Tray"

INCH = 25.4
HOLE = Tolerance.deviation(0.238 * INCH, -0.001 * INCH, 0.005 * INCH)   # Ø6.045 +0.127/-0.025 -> 6.020..6.172
PIN = Tolerance.iso(6.0, "f7")                                          # 5.978..5.990, worst-case clearance 0.030
PIN_CAD = 5.984                                                         # mid of the f7 zone
POS_2X = 0.023 * INCH                                                  # Ø0.584 at MMC to A|B|C
GO = HOLE.lower - POS_2X                                                # virtual condition 5.436
NOGO = HOLE.upper                                                       # LMC 6.172

B_XY = (142.875, -98.425)
C_XY = (-142.875, -98.425)
GAUGE_XY = (142.875, 98.425)

Z_PLATE = -40.0          # base plate top; the part's datum A is Z = 0
FLANGE_TOP = 10.16       # flat band on top of the flange, X 147.5..152.7 on the long sides
PADS = {"Pad_A1": (-146.0, 0.0), "Pad_A2": (146.0, -55.0), "Pad_A3": (146.0, 55.0)}
CLAMP_X = 150.1          # Ø5 clamp tip on the flange band, above the pad
PIVOT_X = 188.0
PIVOT_Z = 8.0            # below the clamp tip (Z 10.16)


def _cyl(r, z0, z1, x, y):
    """Vertical cylinder of radius r from z0 to z1 at (x, y)."""
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(r, z1 - z0)


def _box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def _pin(x, y, diamond):
    """Locating pin: Ø12 shoulder stops 2 mm below datum A, Ø5.984 working section through the flange."""
    shoulder = _cyl(6.0, Z_PLATE, -2.0, x, y)
    work = _cyl(PIN_CAD / 2, -2.0, FLANGE_TOP + 0.6, x, y)
    if diamond:
        # relieved along B->C (X): keep a 2.4 mm wide land in X, full radius in Y
        work = work & _box(x - 1.2, x + 1.2, y - 4, y + 4, -2.0, FLANGE_TOP + 0.6)
    return shoulder + work


def _clamp(name, y, side, pivot_z=PIVOT_Z):
    """Swing-arm clamp from outside the part. side = +1 (right, +X) or -1 (left). Returns (bodies, pivot)."""
    s = side
    px = s * PIVOT_X
    x0, x1 = sorted((s * 180, s * 196))
    post = _box(x0, x1, y - 7, y + 7, Z_PLATE, 0.0)
    ear_top = max(16.0, pivot_z + 8.0)
    ears = [_box(x0, x1, y + 4, y + 7, 0.0, ear_top), _box(x0, x1, y - 7, y - 4, 0.0, ear_top)]
    pin = Pos(px, y, pivot_z) * Rot(90, 0, 0) * Cylinder(2.5, 14.0)
    base = post + ears[0] + ears[1] + pin
    # horizontal arm over the flange plus a lug down to the pivot; the pivot sits below the tip, so every point of
    # the arm moves away from the part as it opens (a pivot above the tip swings the tip into the tray wall)
    arm = _box(*sorted((s * 147.6, s * 196)), y - 4, y + 4, 20.0, 28.0)
    if pivot_z - 5.0 < 20.0:
        arm = arm + _box(*sorted((s * 182, s * 194)), y - 4, y + 4, pivot_z - 5.0, 20.0)
    arm = arm - Pos(px, y, pivot_z) * Rot(90, 0, 0) * Cylinder(2.5, 10.0)
    tip = _cyl(2.5, FLANGE_TOP, 20.0, s * CLAMP_X, y)
    handle = _cyl(4.0, 28.0, 70.0, s * 192.0, y)
    bodies = {f"{name}_Base": base, f"{name}_Arm": arm, f"{name}_Tip": tip, f"{name}_Handle": handle}
    pivot = ((px, y, pivot_z), (0, s, 0))
    return bodies, pivot


def build_bodies(pivot_z: float = PIVOT_Z) -> tuple[dict, dict]:
    part = max(import_step(STEP).solids(), key=lambda x: x.volume)
    bodies = {PART: part}
    bodies["Base_Plate"] = _box(-215, 215, -135, 135, Z_PLATE - 20, Z_PLATE) - \
        _cyl(10.0, Z_PLATE - 21, Z_PLATE + 1, *GAUGE_XY)
    for i, (x, y) in enumerate([(-195, -115), (195, -115), (-195, 115), (195, 115)], start=1):
        bodies[f"Foot_{i}"] = _cyl(15.0, Z_PLATE - 75, Z_PLATE - 20, x, y)
    for name, (x, y) in PADS.items():
        bodies[name] = _cyl(10.0, Z_PLATE, Z_PLATE + 8, x, y) + _cyl(6.0, Z_PLATE + 8, 0.0, x, y)
    bodies["Pin_B_Round"] = _pin(*B_XY, diamond=False)
    bodies["Pin_C_Diamond"] = _pin(*C_XY, diamond=True)
    pivots = {}
    for name, y, side in (("Clamp_1", 0.0, -1), ("Clamp_2", -55.0, 1), ("Clamp_3", 55.0, 1)):
        b, pivots[name] = _clamp(name, y, side, pivot_z)
        bodies.update(b)
    # gauge: block with a Ø16 bore, Ø10 F7 bushing, GO/NO-GO pin shown fully inserted
    gx, gy = GAUGE_XY
    bodies["Gauge_Block"] = _box(gx - 13, gx + 13, gy - 13, gy + 13, Z_PLATE, -6.0) - _cyl(8.0, Z_PLATE - 1, -5.0, gx, gy)
    bodies["Gauge_Bushing"] = _cyl(8.0, Z_PLATE, -6.0, gx, gy) - _cyl(10.022 / 2, Z_PLATE - 1, -5.0, gx, gy)
    bodies["Gauge_GO_NOGO"] = (_cyl(GO / 2, -7.0, 13.0, gx, gy) + _cyl(9.993 / 2, -66.0, -7.0, gx, gy)
                               + _cyl(8.0, -84.0, -66.0, gx, gy) + _cyl(NOGO / 2, -96.0, -84.0, gx, gy))
    return bodies, pivots


def fixture(f: Fixture | None = None, pivots: dict | None = None) -> Fixture:
    if f is None:
        bodies, pivots = build_bodies()
        f = Fixture.from_bodies(bodies, part=PART)
    f = (f.base("Base_Plate")
         .datum_a(list(PADS))
         # the drawing references B and C regardless of feature size (no MMC modifier), so the report notes the
         # shift a fixed pin allows; expanding pins would simulate these datums exactly
         .round_pin("Pin_B_Round", hole=HOLE, pin=PIN, datum_ref="RFS")
         .diamond_pin("Pin_C_Diamond", hole=HOLE, pin=PIN, datum_ref="RFS"))
    for name, pivot in pivots.items():
        f = f.clamp(f"{name}_Tip", arm=[f"{name}_Arm", f"{name}_Handle"], pivot=pivot, opening=100)
    return (f.removal(direction=(0, 0, 1), distance=60)
            .go_nogo_gauge("Gauge_GO_NOGO", direction=(0, 0, 1), retract=20, stroke=20, go_dia=GO, nogo_dia=NOGO,
                           hole=HOLE, position_tol=POS_2X, bushing="Gauge_Bushing",
                           nogo_end="reversed"))  # NO-GO behind the collar: pull the gauge and try it by hand


_BODIES, _PIVOTS = build_bodies()
assembly = fixture(Fixture.from_bodies(_BODIES, part=PART), _PIVOTS).assembly()


# --- intentional fault variants -------------------------------------------------------------

def high_pivot():
    """Clamp pivot above the clamp tip (Z 24 instead of 8): the tip swings into the tray wall as the clamp opens."""
    bodies, pivots = build_bodies(pivot_z=24.0)
    return fixture(Fixture.from_bodies(bodies, part=PART), pivots).assembly()


def loose_pin():
    """Pin B declared Ø5.7 g6 with a 0.1 mm limit on datum shift: the part can shift 0.48 mm around the pin."""
    from dataclasses import replace
    fx = fixture(Fixture.from_bodies(_BODIES, part=PART), _PIVOTS)
    pins = tuple(replace(p, pin=Tolerance.iso(5.7, "g6"), max_shift=0.1) if p.name == "Pin_B_Round" else p
                 for p in fx._pins)
    return replace(fx, _pins=pins).assembly()

