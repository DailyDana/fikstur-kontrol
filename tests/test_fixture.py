"""Passing and failing cases for every check on the synthetic fixture."""

from dataclasses import replace

import pytest
import synthetic as s
from cad_khana.mechanism.assertions import evaluate

from fixture_check import Fixture, Tolerance


def results(f: Fixture) -> dict[str, object]:
    return {r.name: r for r in evaluate(f.assembly())}


def failed(f: Fixture) -> set[str]:
    return {name for name, r in results(f).items() if not r.passed}


def value(f: Fixture, name: str) -> float:
    return results(f)[name].value


@pytest.fixture(scope="module")
def good():
    return results(s.fixture())


def test_good_fixture_all_green(good):
    failures = [name for name, r in good.items() if not r.passed]
    assert failures == []
    assert len(good) > 40  # not vacuously green


@pytest.mark.parametrize("name, expected", [
    ("F1:pads_in_contact", 3),
    ("F6:constrained_dof", 6),
    ("pin:Pin_B:cad_dia_clearance_mm", 0.032),
    ("pin:Pin_B:shoulder_gap_mm", 2.0),
    ("pin:Pin_B:worst_case_clearance_mm", 0.025),
    ("pin:Pin_C:diamond_relief_mm", 8.968 - 5.5),
    ("pin:Pin_C:diamond_orientation_error_deg", 0.0),
    ("flush:Block:+Z", 0.0),
    ("K8:floating_bodies", 0),
    ("F3:clamp_over_pad_mm:Clamp_Pad", 8.0),   # clamping center on pad center, pad Ø16
])
def test_measured_values(good, name, expected):
    assert good[name].value == pytest.approx(expected, abs=2e-3)


def test_support_triangle_and_stability(good):
    # A1(-40,-30) A2(40,-30) A3(0,40): shortest altitude is to edge A1-A3
    assert good["F1:support_triangle_min_height_mm"].value == pytest.approx(80 * 70 / (80.6226), abs=0.01)
    assert good["warn:F2:stability_margin_mm"].value > 0


def test_reversed_diamond_fails():
    g = s.bodies()
    g["Pin_C"] = s.diamond_pin(narrow="Y")
    f = failed(s.fixture(g))
    assert "pin:Pin_C:diamond_relief_mm" in f
    assert "pin:Pin_C:diamond_orientation_error_deg" in f


def test_cylindrical_pin_declared_as_diamond_fails():
    g = s.bodies()
    g["Pin_C"] = s.round_pin(30, 0)
    f = failed(s.fixture(g))
    assert "pin:Pin_C:diamond_relief_mm" in f


def test_two_round_pins_overconstrained():
    f = s.fixture()
    pins = tuple(replace(p, diamond=False) for p in f._pins)
    f = replace(f, _pins=pins)
    assert value(f, "F6:constrained_dof") == 7
    assert "F6:constrained_dof" in failed(f)


def test_missing_pad_contact():
    g = s.bodies()
    g["Pad_A3"] = s.move(g["Pad_A3"], z=-0.5)   # pad 0.5 mm short
    f = failed(s.fixture(g))
    assert "F1:pads_in_contact" in f
    assert "contact:Pad_A3" in f


def test_tight_pin_fails_worst_case():
    f = s.fixture()
    pins = tuple(replace(p, pin=Tolerance.iso(9.0, "js6")) if p.name == "Pin_B" else p for p in f._pins)
    fail = failed(replace(f, _pins=pins))
    assert "pin:Pin_B:worst_case_clearance_mm" in fail


def test_clamp_off_pad():
    g = s.bodies()
    g["Clamp_Pad"] = s.move(g["Clamp_Pad"], x=12)
    assert "F3:clamp_over_pad_mm:Clamp_Pad" in failed(s.fixture(g))


def test_gap_and_flush():
    g = s.bodies()
    g["Block"] = s.move(g["Block"], x=-1, z=0.5)
    f = failed(s.fixture(g))
    assert "gap:Block" in f
    assert "flush:Block:+Z" in f


def test_removal_path_obstruction():
    g = s.bodies()
    g["Obstacle"] = s._box(-10, 10, -10, 10, 50, 55) + s._box(-10, 10, 70, 80, 20, 55)  # bridge attached to base
    f = failed(s.fixture(g))
    assert "clear_of:removal_path:30mm>=0" in f
    assert "K8:floating_bodies" not in f


def test_floating_body_and_acceptance():
    g = s.bodies()
    g["Floating"] = s._box(80, 90, 80, 90, 30, 40)
    f = s.fixture(g)
    assert "K8:floating_bodies" in failed(f)
    f2 = f.accept_floating("Floating", "test")
    res = results(f2)
    assert res["K8:floating_bodies"].passed
    assert res["warn:K8:accepted_floating:Floating"].value == 1


def test_known_interference():
    g = s.bodies()
    g["Extra"] = s._box(60, 70, 60, 70, 15, 30)  # embedded 5 mm into the base
    f = s.fixture(g)
    assert any(name.startswith("no_interference:") for name in failed(f))
    f2 = f.known_interference("Base", "Extra", "test")
    assert not any(name.startswith("no_interference:") for name in failed(f2))
    assert results(f2)["known_interference:Base/Extra"].passed


def test_invalid_declarations():
    f = Fixture.from_bodies(s.bodies(), part="Part")
    with pytest.raises(KeyError):
        f.datum_a(["Pad_A1", "Pad_X", "Pad_A3"])
    with pytest.raises(KeyError):
        f.moving("Missing_*")
    with pytest.raises(KeyError):
        Fixture.from_bodies(s.bodies(), part="Missing")
    with pytest.raises(ValueError):
        f.datum_a(list(s.PADS)).clamp("Pad_A1").assembly()   # pad declared as both datum and clamp
    with pytest.raises(ValueError):
        f.go_nogo_gauge("Pin_B", direction=(0, 0, 1), stroke=1, go_dia=1, modifier="LMC")


def test_immutability():
    f = Fixture.from_bodies(s.bodies(), part="Part")
    f2 = f.datum_a(list(s.PADS))
    assert f._datum is None and f2._datum is not None
    with pytest.raises(KeyError):
        f.add_body("Base", s._box(0, 1, 0, 1, 0, 1))


# --- "False green" scenarios from Gemini's final review -------------------------------------

def test_shoulder_touching_part_fails():
    g = s.bodies()
    g["Pin_B"] = s._cylinder(-30, 0, 20, 40, 8) + s._cylinder(-30, 0, 40, 46, s.PIN_DIA / 2)
    g["Pad_A1"] = g["Pad_A1"]  # pads unchanged
    f = failed(s.fixture(g))
    assert "pin:Pin_B:shoulder_gap_mm" in f


def test_flush_does_not_match_opposite_face():
    g = s.bodies()
    g["Block"] = s._box(63, 80, -20, 20, 20, 40)   # top Z=40 = plane of the part's BOTTOM face
    assert "flush:Block:+Z" in failed(s.fixture(g))


def test_three_diamond_pins_layout_fails():
    g = s.bodies()
    g["Pin_D"] = s.diamond_pin(0, -20)
    g["Part"] = g["Part"] - s._cylinder(0, -20, 39, 43, 4.5)
    f = s.fixture(g).diamond_pin("Pin_D", hole=s.HOLE, pin=s.PIN)
    f = replace(f, _pins=tuple(replace(p, diamond=True) for p in f._pins))
    assert "F6:pin_layout" in failed(f)


def test_duplicate_pin_raises():
    with pytest.raises(ValueError):
        s.fixture().round_pin("Pin_B").assembly()


def test_offset_pin_fails_worst_case_clearance():
    g = s.bodies()
    g["Pin_B"] = s.round_pin(-30 + 0.015, 0)   # no interference in CAD (radial clearance 0.016)
    f = failed(s.fixture(g))
    assert "pin:Pin_B:worst_case_clearance_with_offset_mm" in f
    assert not any(a.startswith("no_interference:") for a in f)


def test_diamond_pin_declared_as_round_fails():
    f = s.fixture()
    f = replace(f, _pins=tuple(replace(p, diamond=False) if p.name == "Pin_C" else p for p in f._pins))
    assert "pin:Pin_C:roundness" in failed(f)


def test_slot_perpendicular_to_bc_fails():
    from build123d import Pos, Rot, SlotCenterToCenter, extrude
    g = s.bodies()
    p = s._box(-60, 60, -50, 50, 40, 42) - s._cylinder(-30, 0, 39, 43, 4.5)
    p -= Pos(30, 0, 39) * Rot(0, 0, 90) * extrude(SlotCenterToCenter(3.0, 9.0), 4)
    g["Part"] = p
    assert "pin:Pin_C:slot_orientation_error_deg" in failed(s.fixture(g))


def test_inverted_gravity_unstable():
    f = replace(s.fixture(), gravity=(0, 0, 1), strict_stability=True)
    assert "F2:stability_margin_mm" in failed(f)


def test_clamp_pressing_overhanging_flange_fails():
    g = s.bodies()
    # add a tab 20 mm above A1 (air gap underneath) and clamp onto it
    g["Part"] = g["Part"] + s._box(-50, -30, -40, -20, 60, 62) + s._box(-60, -58, -40, -20, 42, 62)
    g["Clamp_Pad"] = s._cylinder(-40, -30, 62, 72, 6)
    g["Clamp_Arm"] = s._box(-45, -35, -65, -25, 72, 77)
    g["Clamp_Post"] = s._box(-45, -35, -65, -55, 20, 72)
    f = failed(s.fixture(g))
    assert "F3:clamp_column_filled:Clamp_Pad" in f


def test_gauge_missing_hole_fails():
    g = s.bodies()
    # Ø6 hole in a vertical side wall (2 mm flange on part edge) and Ø4 GO gauge along X
    g["Part"] = g["Part"] + s._box(-60, -58, -10, 10, 42, 60)
    from build123d import Pos, Rot
    g["Part"] = g["Part"] - Pos(-59, 0, 52) * Rot(0, 90, 0) * s.Cylinder(3, 4)
    gauge = Pos(-75, 0, 52) * Rot(0, 90, 0) * s.Cylinder(2, 20)
    shifted = Pos(-75, 2, 52) * Rot(0, 90, 0) * s.Cylinder(2, 20)
    for body, should_fail in ((gauge, False), (shifted, True)):
        g["GO"] = body
        f = s.fixture(g).go_nogo_gauge("GO", direction=(1, 0, 0), stroke=10, go_dia=4.0)
        fail = failed(f)
        hits = {a for a in fail if a.startswith("gauge:GO")}
        assert bool(hits) == should_fail, fail


# --- K9: clamp opening arc ------------------------------------------------------------------

def _opening_clamp(g):
    f = s.fixture(g)
    c = replace(f._clamps[0], group=None, arm=("Clamp_Arm",), pivot=((-40, -65, 57), (1, 0, 0)), opening=90.0)
    return replace(f, _clamps=(c,))


def test_clamp_opening_clear():
    assert value(_opening_clamp(s.bodies()), "K9:clamp_opening_deg:Clamp_Pad") == 90


def test_clamp_opening_obstructed():
    g = s.bodies()
    g["Ceiling"] = s._box(-50, -30, -35, -25, 75, 80)   # above the arm tip's opening arc
    f = _opening_clamp(g)
    r = results(f)["K9:clamp_opening_deg:Clamp_Pad"]
    assert not r.passed
    assert 20 < r.value < 35   # analytical: arm tip hits the ceiling at ~27°


def test_clamp_opening_rotates_group_too():
    g = s.bodies()
    g["Ceiling"] = s._box(-50, -30, -35, -25, 75, 80)
    f = s.fixture(g)
    c = replace(f._clamps[0], group="Clamp_Arm", arm=(), pivot=((-40, -65, 57), (1, 0, 0)), opening=90.0)
    assert "K9:clamp_opening_deg:Clamp_Pad" in failed(replace(f, _clamps=(c,)))


def test_clamp_pivot_without_arm_raises_and_skipped_warning():
    f = Fixture.from_bodies(s.bodies(), part="Part")
    with pytest.raises(ValueError):
        f.clamp("Clamp_Pad", pivot=((0, 0, 0), (1, 0, 0)))
    assert "warn:K9:skipped:Clamp_Pad" in results(s.fixture())
