"""Planar 3-2-1: datum A pads, two datum B rest buttons on a side face, one datum C button on the end face.

Part: block X -50..50, Y -30..30, Z 40..70. Pads under it (Z 20..40). B buttons touch the -Y face (Y = -30),
C button touches the -X face (X = -50). Pushers press from +Y (between the B buttons) and +X (opposite C).
"""

import pytest
from build123d import Box, Cylinder, Pos, Rot
from cad_khana.mechanism.assertions import evaluate

from fixture_check import Fixture


def _box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def _post(x, y):
    return _box(x - 4, x + 4, y - 4, y + 4, 20, 50)


def bodies(**over):
    g = {
        "Part": _box(-50, 50, -30, 30, 40, 70),
        "Base": _box(-90, 90, -70, 70, 0, 20),
        "Pad_1": Pos(-35, -15, 30) * Cylinder(6, 20),
        "Pad_2": Pos(35, -15, 30) * Cylinder(6, 20),
        "Pad_3": Pos(0, 20, 30) * Cylinder(6, 20),
        # datum B buttons: Ø10 faces at Y = -30, pointing +Y, on posts standing on the base
        "B_1": _post(-30, -40) + Pos(-30, -33, 55) * Rot(90, 0, 0) * Cylinder(5, 6),
        "B_2": _post(30, -40) + Pos(30, -33, 55) * Rot(90, 0, 0) * Cylinder(5, 6),
        # datum C button: face at X = -50, pointing +X
        "C_1": _post(-60, 0) + Pos(-53, 0, 55) * Rot(0, 90, 0) * Cylinder(5, 6),
        # pushers: B pusher at X = 0 on the +Y face, C pusher on the +X face
        "Push_B": _post(0, 40) + Pos(0, 33, 55) * Rot(90, 0, 0) * Cylinder(4, 6),
        "Push_C": _post(60, 0) + Pos(53, 0, 55) * Rot(0, 90, 0) * Cylinder(4, 6),
    }
    g.update(over)
    return g


def fixture(g=None, *, b=("B_1", "B_2"), c="C_1", push_b="Push_B") -> Fixture:
    f = (Fixture.from_bodies(g or bodies(), part="Part")
         .base("Base")
         .datum_a(["Pad_1", "Pad_2", "Pad_3"]))
    if b:
        f = f.datum_b(list(b))
    if c:
        f = f.datum_c(c)
    if push_b:
        f = f.clamp(push_b, against="B")
    return f.clamp("Push_C", against="C")


def results(f):
    return {r.name: r for r in evaluate(f.assembly())}


def failed(f):
    return {n for n, r in results(f).items() if not r.passed}


def test_planar_321_all_green():
    r = results(fixture())
    assert [n for n, x in r.items() if not x.passed] == []
    assert r["F6:constrained_dof"].value == 6
    assert r["F1B:button_spacing_mm"].value == pytest.approx(60.0)
    assert r["F3:pusher_over_datum_B_mm:Push_B"].value == pytest.approx(5.0, abs=0.05)  # limited by button radius


def test_missing_datum_c_under_constrained():
    g = bodies()
    del g["C_1"]
    f = (Fixture.from_bodies(g, part="Part").base("Base").datum_a(["Pad_1", "Pad_2", "Pad_3"])
         .datum_b(["B_1", "B_2"]).clamp("Push_B", against="B").clamp("Push_C"))
    bad = failed(f)
    assert {"F6:constrained_dof", "F6:pin_layout"} <= bad


def test_button_off_the_face():
    # B_2 pulled back 1 mm: only one B button touches
    g = bodies(B_2=_post(30, -41) + Pos(30, -34, 55) * Rot(90, 0, 0) * Cylinder(5, 6))
    bad = failed(fixture(g))
    assert "F1B:buttons_in_contact" in bad
    assert "F6:constrained_dof" in bad


def test_pusher_outside_button_span_tilts_part():
    g = bodies(Push_B=_post(45, 40) + Pos(45, 33, 55) * Rot(90, 0, 0) * Cylinder(4, 6))
    assert "F3:pusher_over_datum_B_mm:Push_B" in failed(fixture(g))


def test_round_pin_and_datum_b_face_claim_the_same_dof():
    """A round pin plus a datum B face: two 2-DOF locators, layout fails and the DOF count exceeds 6."""
    from dataclasses import replace

    from cad_khana.mechanism.assembly import Assembly

    from fixture_check.fixture import _Pin
    f = replace(fixture(), _pins=(_Pin("Push_B", False, None, None, None, 1.0),))
    a = f._dof_checks(Assembly(), {"Pad_1": 1, "Pad_2": 1, "Pad_3": 1}, {"Push_B": {}},
                      {"B": {"B_1": 1, "B_2": 1}, "C": {"C_1": 1}})
    r = {x.name: x for x in evaluate(a)}
    assert not r["F6:pin_layout"].passed
    assert r["F6:constrained_dof"].value == 8


def test_invalid_declarations():
    f = Fixture.from_bodies(bodies(), part="Part")
    with pytest.raises(ValueError):
        f.datum_b(["B_1"])
    with pytest.raises(ValueError):
        f.clamp("Push_B", against="D")
