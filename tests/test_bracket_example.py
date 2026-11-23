"""Real fixture (examples/bracket_fixture_assembly.step): design passes, each intentional fault fails on the
expected assertion."""

import importlib.util
from pathlib import Path

import pytest
from cad_khana.mechanism.assertions import evaluate

EXAMPLE = Path(__file__).resolve().parents[1] / "examples" / "bracket_fixture.py"
STEP = EXAMPLE.with_name("bracket_fixture_assembly.step")
pytestmark = pytest.mark.skipif(not STEP.exists(), reason="example STEP not found")


@pytest.fixture(scope="module")
def example():
    spec = importlib.util.spec_from_file_location("bracket_fixture", EXAMPLE)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def failed(asm) -> set[str]:
    return {r.name for r in evaluate(asm) if not r.passed}


def test_real_design_all_green(example):
    res = evaluate(example.assembly)
    assert [r.name for r in res if not r.passed] == []
    assert len(res) > 300


@pytest.mark.parametrize("variant, expected", [
    ("covered_top", {"clear_of:removal_path:65mm>=0", "clear:Hatali_Ust_Blok"}),
    ("reversed_diamond", {"pin:Datum_C_Elmas_Pim:diamond_relief_mm", "pin:Datum_C_Elmas_Pim:diamond_orientation_error_deg"}),
    ("clamp_off_pad", {"F3:clamp_over_pad_mm:Mandalli_Klemp_Sol_Pabuc"}),
    ("tight_pin", {"pin:Datum_B_Yuvarlak_Pim:worst_case_clearance_mm"}),
    ("floating_tower", {"K8:floating_bodies"}),
    ("missing_pin", {"F6:constrained_dof"}),
    ("shifted_bushing", {"clear_of:gauge:Gec_Gecmez_Pim_GO750_NOGO810:path_clear>=0"}),
])
def test_faulty_variant_fails(example, variant, expected):
    f = failed(getattr(example, variant)())
    assert expected <= f, f"missing expected failures: {expected - f}; failed: {sorted(f)}"
