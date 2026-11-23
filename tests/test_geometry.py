import math

import pytest
from build123d import Box, Pos, Solid

from fixture_check import geometry as geo
from fixture_check.fixture import _convex_hull, _direction_name, _hull_min_width


def test_to_part_preserves_volume():
    box = Box(10, 10, 10)
    solid = box.solids()[0]
    assert isinstance(solid, Solid)
    assert geo.to_part(solid).volume == pytest.approx(1000)


def test_triangle_min_height():
    assert geo.triangle_min_height((0, 0), (10, 0), (0, 10)) == pytest.approx(10 / math.sqrt(2) * 1)
    assert geo.triangle_min_height((0, 0), (5, 0), (10, 0)) == 0


@pytest.mark.parametrize("p, expected", [((5, 5), 5), ((1, 5), 1), ((-2, 5), -2), ((5, 12), -2)])
def test_polygon_edge_margin(p, expected):
    square = [(0, 0), (10, 0), (10, 10), (0, 10)]
    assert geo.polygon_edge_margin(p, square) == pytest.approx(expected)
    assert geo.polygon_edge_margin(p, list(reversed(square))) == pytest.approx(expected)


def test_hull_and_min_width():
    hull = _convex_hull([(0, 0), (10, 0), (10, 4), (0, 4), (5, 2)])
    assert len(hull) == 4
    assert _hull_min_width(hull) == pytest.approx(4)


def test_axis_angle_diff():
    assert geo.axis_angle_diff(10, 170) == pytest.approx(20)
    assert geo.axis_angle_diff(0, 90) == pytest.approx(90)
    assert geo.axis_angle_diff(179, 1) == pytest.approx(2)


def test_contact_patch_and_edge_margin():
    bottom = Pos(0, 0, 5) * Box(20, 20, 10)    # top face at Z=10, 20x20
    top = Pos(5, 0, 11) * Box(10, 30, 2)       # Z 10..12, X 0..10
    patch = geo.contact_patch(bottom, top)
    assert patch is not None
    assert patch.area == pytest.approx(200)    # overlap X 0..10, Y -10..10
    assert (patch.center.X, patch.center.Y, patch.center.Z) == pytest.approx((5, 0, 10))
    assert patch.normal.Z == pytest.approx(1)
    assert patch.support_area == pytest.approx(400)
    assert geo.face_edge_margin(patch.center, patch.support_outline) == pytest.approx(5)
    assert geo.contact_patch(bottom, Pos(0, 0, 20) * Box(5, 5, 5)) is None


def test_direction_name():
    assert _direction_name((0, 1, 0)) == "+Y"
    assert _direction_name((0, 0, -1)) == "-Z"
    assert _direction_name((0.6, 0.8, 0)) == "(0.6,0.8,0)"
