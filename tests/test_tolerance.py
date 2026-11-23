import pytest

from fixture_check.tolerance import Fit, Tolerance


@pytest.mark.parametrize(
    "nominal, zone, lower, upper",
    [
        (9, "e7", 8.960, 8.975),
        (12, "F7", 12.016, 12.034),
        (12, "g6", 11.983, 11.994),
        (18, "n6", 18.012, 18.023),   # 18 inclusive: 10..18 step
        (18, "H7", 18.000, 18.018),
        (25, "H7", 25.000, 25.021),
        (25, "f7", 24.959, 24.980),
        (9, "H9", 9.000, 9.036),
        (10, "k6", 10.001, 10.010),
        (10, "p6", 10.015, 10.024),
        (6, "h6", 5.992, 6.000),
        (40, "js7", 39.988, 40.012),   # IT7=25 odd -> ±12
        (9, "js6", 8.9955, 9.0045),    # IT6=9, below IT7 not rounded
        (50, "G7", 50.009, 50.034),
        (3, "h7", 2.990, 3.000),      # upper bound of first step
        (10, "k8", 10.000, 10.022),   # k coarser than IT7: ei = 0
    ],
)
def test_iso_zones(nominal, zone, lower, upper):
    t = Tolerance.iso(nominal, zone)
    assert t.lower == pytest.approx(lower, abs=1e-9)
    assert t.upper == pytest.approx(upper, abs=1e-9)


@pytest.mark.parametrize("nominal, zone", [(0, "H7"), (600, "H7"), (10, "H4"), (10, "x7"), (10, "K7"), (10, "7")])
def test_unsupported_raises(nominal, zone):
    with pytest.raises(ValueError):
        Tolerance.iso(nominal, zone)


def test_fit_limits():
    fit = Fit(hole=Tolerance.iso(9, "H9"), pin=Tolerance.iso(9, "e7"))
    assert fit.min_clearance == pytest.approx(0.025)
    assert fit.max_clearance == pytest.approx(0.076)


def test_explicit_limits_and_plus_minus():
    assert Tolerance.plus_minus(8.0, 0.1) == Tolerance(7.9, 8.1, "8 ±0.1")
    with pytest.raises(ValueError):
        Tolerance.limits(2.0, 1.0)
    assert Tolerance.limits(1, 2).contains(2.0000001)
