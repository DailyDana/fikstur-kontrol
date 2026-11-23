"""Small, hand-measurable synthetic fixture for tests.

Base Z 0..20; three pads Z 20..40; sheet part 120x100x2, Z 40..42, Ø9 hole at (-30, 0) and
9x12 slot at (30, 0, along X); round and diamond pins Ø8.968 (shoulder top Z=38, 2 mm below part);
clamp pad centered directly over pad A1; contour block X 63..80 (3 mm gap), top at Z=42 (flush with part).
"""

from build123d import Box, Compound, Cylinder, Location, Pos, SlotCenterToCenter, extrude

from fixture_check import Fixture, Tolerance

PIN_DIA = 8.968
PADS = {"Pad_A1": (-40, -30), "Pad_A2": (40, -30), "Pad_A3": (0, 40)}
HOLE = Tolerance.deviation(9.0, 0.0, 0.10)
PIN = Tolerance.iso(9.0, "e7")


def _box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def _cylinder(x, y, z0, z1, r):
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(r, z1 - z0)


def part():
    p = _box(-60, 60, -50, 50, 40, 42)
    p -= _cylinder(-30, 0, 39, 43, 4.5)
    p -= Pos(30, 0, 39) * extrude(SlotCenterToCenter(3.0, 9.0), 4)
    return p


def round_pin(x=-30, y=0):
    return _cylinder(x, y, 20, 38, 8) + _cylinder(x, y, 38, 46, PIN_DIA / 2)


def diamond_pin(x=30, y=0, narrow="X"):
    working = _cylinder(x, y, 38, 46, PIN_DIA / 2)
    cutter = _box(x - 2.75, x + 2.75, y - 10, y + 10, 37, 47) if narrow == "X" else \
        _box(x - 10, x + 10, y - 2.75, y + 2.75, 37, 47)
    return _cylinder(x, y, 20, 38, 8) + (working & cutter)


def bodies() -> dict:
    g = {
        "Base": _box(-100, 100, -100, 100, 0, 20),
        "Part": part(),
        "Pin_B": round_pin(),
        "Pin_C": diamond_pin(),
        "Clamp_Pad": _cylinder(-40, -30, 42, 52, 6),
        "Clamp_Arm": _box(-45, -35, -65, -25, 52, 57),
        "Clamp_Post": _box(-45, -35, -65, -55, 20, 52),
        "Block": _box(63, 80, -20, 20, 20, 42),
    }
    for name, (x, y) in PADS.items():
        g[name] = _cylinder(x, y, 20, 40, 8)
    return g


def fixture(g: dict | None = None) -> Fixture:
    f = Fixture.from_bodies(g or bodies(), part="Part")
    return (f.base("Base")
            .datum_a(list(PADS))
            .round_pin("Pin_B", hole=HOLE, pin=PIN)
            .diamond_pin("Pin_C", hole=HOLE, pin=PIN)
            .clamp("Clamp_Pad", group="Clamp_*")
            .contour("Block", gap=3.0)
            .flush("Block", direction=(0, 0, 1))
            .removal(direction=(0, 0, 1), distance=30))


def move(shape, x=0.0, y=0.0, z=0.0):
    return shape.moved(Location((x, y, z)))


__all__ = ["Compound", "Cylinder", "bodies", "diamond_pin", "fixture", "move", "round_pin"]
