"""fixture-check specification for the sheet-metal bracket checking fixture in bracket_fixture_assembly.step.

    khana check examples/bracket_fixture.py               # actual design
    khana check examples/bracket_fixture.py:<variant>     # intentional fault variants, below

Body names in the STEP file are Turkish (the fixture was modelled first). Part holes use sheet-metal manufacturing tolerances
(Ø9 +0.10/0, slot 9 +0.10/0, flange hole Ø8 ±0.10, position Ø0.40 MMC); pins are Ø9 e7.
"""

from pathlib import Path

from build123d import Box, Location, Pos

from fixture_check import Fixture, Tolerance

STEP = Path(__file__).resolve().with_name("bracket_fixture_assembly.step")
PART = "Sac_Braket_DC01_2mm"

HOLE_B = Tolerance.deviation(9.0, 0.0, 0.10)
SLOT_C = Tolerance.deviation(9.0, 0.0, 0.10)
PIN = Tolerance.iso(9.0, "e7")
FLANGE_HOLE = Tolerance.plus_minus(8.0, 0.10)


def fixture(f: Fixture | None = None) -> Fixture:
    f = f or Fixture.step(STEP, part=PART)
    return (
        f.base("Taban_Plakasi_Al5083_20mm")
        .datum_a(["Datum_A_Pedi_A1_ArkaSol", "Datum_A_Pedi_A2_ArkaSag", "Datum_A_Pedi_A3_On"])
        .round_pin("Datum_B_Yuvarlak_Pim", hole=HOLE_B, pin=PIN)
        .diamond_pin("Datum_C_Elmas_Pim", hole=SLOT_C, pin=PIN)
        # one toggle clamp over each pad; clamp bases are fixed, arm, pad and handle swing open.
        # The rear clamps come in from the sides so their arms clear the bracket's return tab.
        .clamp("Mandalli_Klemp_Sol_Pabuc", arm=["Mandalli_Klemp_Sol_Kol", "Mandalli_Klemp_Sol_Elcik"],
               pivot=((-88, 24, 102), (0, -1, 0)), opening=100)
        .clamp("Mandalli_Klemp_Sag_Pabuc", arm=["Mandalli_Klemp_Sag_Kol", "Mandalli_Klemp_Sag_Elcik"],
               pivot=((88, 24, 102), (0, 1, 0)), opening=100)
        .clamp("Mandalli_Klemp_On_Pabuc", arm=["Mandalli_Klemp_On_Kol", "Mandalli_Klemp_On_Elcik"],
               pivot=((0, -66, 102), (1, 0, 0)), opening=100)
        .contour("Kontur_Blogu_Sol_3mm_Flush", gap=3.0)
        .contour("Kontur_Blogu_Sag_3mm_Flush", gap=3.0)
        .contour("Kontur_Blogu_Kulak_3mm_Flush", gap=3.0)
        .flush("Kontur_Blogu_Sol_3mm_Flush", direction=(0, -1, 0), opposed=True)  # front face flush with outer flange face
        .flush("Kontur_Blogu_Sag_3mm_Flush", direction=(0, -1, 0), opposed=True)  # front face flush with outer flange face
        .flush("Kontur_Blogu_Kulak_3mm_Flush", direction=(0, 0, 1))
        .flush("Kontur_Blogu_Sol_3mm_Flush", direction=(0, 0, 1), point=(-44, 39, 127))   # top of left wing Z=127
        .flush("Kontur_Blogu_Sag_3mm_Flush", direction=(0, 0, 1), point=(44, 39, 127))    # top of right wing Z=127
        .removal(direction=(0, 0, 1), distance=65)
        .go_nogo_gauge("Gec_Gecmez_Pim_GO750_NOGO810", direction=(0, -1, 0), retract=10, stroke=10,
                       go_dia=7.50, nogo_dia=8.10, hole=FLANGE_HOLE, position_tol=0.40,
                       bushing="Kilavuz_Burc_DIN172A_12F7",
                       nogo_end="reversed")  # NO-GO sits behind the stop collar: pull the pin and try it by hand
    )


assembly = fixture().assembly()


# --- intentional fault variants: each must fail on a specific assertion ---------------------

def _base() -> Fixture:
    return Fixture.step(STEP, part=PART)


def covered_top():
    """Bug from Neon's early attempts: a contour block extends over the top of the bracket."""
    block = Pos(0, 25, 140) * Box(40, 30, 10)
    return fixture(_base().add_body("Hatali_Ust_Blok", block)).assembly()


def reversed_diamond():
    """Diamond pin rotated 90°: wide side lies along B→C, leaving no clearance in the slot."""
    f = _base()
    pin = f.bodies["Datum_C_Elmas_Pim"]
    c = pin.center()
    rotated = pin.moved(Location((c.X, c.Y, 0), (0, 0, 1), 90) * Location((-c.X, -c.Y, 0)))
    return fixture(f.replace_body("Datum_C_Elmas_Pim", rotated)).assembly()


def clamp_off_pad():
    """Left clamp pad shifted 14 mm towards the front: presses the sheet where there is no support pad."""
    f = _base()
    pad = f.bodies["Mandalli_Klemp_Sol_Pabuc"].moved(Location((0, -14, 0)))
    return fixture(f.replace_body("Mandalli_Klemp_Sol_Pabuc", pad)).assembly()


def tight_pin():
    """Pin B declared Ø9 js6: worst case puts a Ø9.0045 pin in a Ø9.000 hole, so the part may not fit."""
    from dataclasses import replace
    fx = fixture(_base())
    pins = tuple(replace(p, pin=Tolerance.iso(9.0, "js6")) if p.name == "Datum_B_Yuvarlak_Pim" else p
                 for p in fx._pins)
    return replace(fx, _pins=pins).assembly()


def floating_tower():
    """Bushing tower modeled 5 mm too high: does not touch the base plate."""
    f = _base()
    for name in ["Burc_Tasiyici_Kule", "Kilavuz_Burc_DIN172A_12F7"]:
        f = f.replace_body(name, f.bodies[name].moved(Location((0, 0, 5))))
    return fixture(f).assembly()


def missing_pin():
    """Diamond pin omitted: the part can rotate around pin B (only 5 DOF constrained)."""
    from dataclasses import replace
    fx = fixture(_base())
    return replace(fx, _pins=tuple(p for p in fx._pins if p.diamond is False)).assembly()


def shifted_bushing():
    """Bushing tower, bushing and gauge shifted 1 mm left together: GO pin hits the flange hole wall."""
    f = _base()
    for name in ["Burc_Tasiyici_Kule", "Kilavuz_Burc_DIN172A_12F7", "Gec_Gecmez_Pim_GO750_NOGO810"]:
        f = f.replace_body(name, f.bodies[name].moved(Location((-1, 0, 0))))
    return fixture(f).assembly()
