"""NIST FTC-08 (a real machined part with fillets): the fixture passes, each intentional fault fails as expected."""

import importlib.util
from pathlib import Path

import pytest
from cad_khana.mechanism.assertions import evaluate

EXAMPLE = Path(__file__).resolve().parents[1] / "examples" / "nist_ftc08_fixture.py"
pytestmark = pytest.mark.skipif(not EXAMPLE.with_name("nist_ftc_08_asme1_rc.stp").exists(), reason="NIST STEP not found")


@pytest.fixture(scope="module")
def example():
    spec = importlib.util.spec_from_file_location("nist_ftc08_fixture", EXAMPLE)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def test_design_all_green(example):
    res = {r.name: r for r in evaluate(example.assembly)}
    assert [n for n, r in res.items() if not r.passed] == []
    assert res["F1:pads_in_contact"].value == 3
    assert res["gauge:Gauge_GO_NOGO:F7:go_dia"].value == pytest.approx(5.436, abs=1e-3)
    assert "warn:removal_path:60mm:sampled_step_mm" in res  # the tray has torus faces


@pytest.mark.parametrize("variant, expected", [
    ("high_pivot", {"K9:clamp_opening_deg:Clamp_1_Tip", "K9:clamp_opening_deg:Clamp_2_Tip",
                    "K9:clamp_opening_deg:Clamp_3_Tip"}),
    ("loose_pin", {"pin:Pin_B_Round:datum_shift_mm"}),
])
def test_faulty_variant_fails(example, variant, expected):
    f = {r.name for r in evaluate(getattr(example, variant)()) if not r.passed}
    assert expected <= f, f"missing expected failures: {expected - f}; failed: {sorted(f)}"
