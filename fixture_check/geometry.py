"""Geometry helpers for fixture checks (pure functions on top of build123d)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

from build123d import (
    Compound,
    Face,
    GeomType,
    Part,
    Plane,
    Shape,
    Solid,
    Vector,
    import_step,
)

CONTACT_TOL = 1e-3  # mm, matches cad-khana's default tangent contact tolerance


def to_part(shape: Shape) -> Part:
    """Wrap any shape as a Part. Wrapping a bare Solid directly gives Part.volume == 0,
    so place it inside a Compound first."""
    if isinstance(shape, Part) and shape.wrapped.ShapeType().name != "TopAbs_SOLID":
        return shape
    solids = shape.solids()
    if not solids:
        raise ValueError("shape contains no solid bodies")
    return Part(Compound(list(solids)).wrapped)


def _collect_solids(shape, parent_label: str = "") -> list[tuple[str, Solid]]:
    label = getattr(shape, "label", "") or parent_label
    if isinstance(shape, Solid):
        return [(label, shape)]
    result: list[tuple[str, Solid]] = []
    children = getattr(shape, "children", None) or []
    if children:
        for c in children:
            result.extend(_collect_solids(c, label))
    else:
        result.extend((label, s) for s in shape.solids())
    return result


def step_bodies(path: str | Path) -> dict[str, Part]:
    """Every solid body in a STEP assembly, keyed by label. Unnamed or duplicate labels raise."""
    result: dict[str, Part] = {}
    for label, solid in _collect_solids(import_step(str(path))):
        if not label:
            raise ValueError("STEP contains an unnamed body; every body must be named")
        if label in result:
            raise ValueError(f"STEP contains duplicate body name: {label!r}")
        result[label] = to_part(solid)
    return result


@dataclass(frozen=True)
class ContactPatch:
    """Planar contact region between two bodies."""

    face: Face            # contact patch (overlap of the two faces)
    center: Vector        # area centroid
    area: float           # mm²
    normal: Vector        # unit normal pointing from the supporting body toward the part
    support_area: float   # area of the reference contacting face on the supporting body
    support_outline: Face  # outer boundary of the support face without holes


def contact_patch(support: Shape, part: Shape, tol: float = CONTACT_TOL) -> ContactPatch | None:
    """Overlap between planar faces of `support` touching `part` and opposing faces of `part`.
    With multiple overlaps, returns combined area and area-weighted centroid. None if no contact."""
    patches: list[Face] = []
    normal: Vector | None = None
    for f in support.faces():
        if f.geom_type != GeomType.PLANE or f.distance_to(part) > tol:
            continue
        nf = f.normal_at(f.center())
        for g in part.faces():
            if g.geom_type != GeomType.PLANE or g.distance_to(f) > tol:
                continue
            ng = g.normal_at(g.center())
            if nf.dot(ng) > -0.999:  # must be opposing (opposite normals)
                continue
            # project g onto f's plane and take the overlap
            shift = nf * (Vector(f.center()) - Vector(g.center())).dot(nf)
            common = f.intersect(g.translate(shift))
            if not common:
                continue
            for y in common.faces():
                if y.area > 1e-6:
                    patches.append(y)
                    normal = nf
    if not patches or normal is None:
        return None
    area = sum(y.area for y in patches)
    center = sum((Vector(y.center()) * y.area for y in patches), Vector(0, 0, 0)) / area
    nn = normal.normalized()
    # reference support face: among faces with the same normal, closest to part (topmost), then largest
    candidates = [
        f for f in support.faces()
        if f.geom_type == GeomType.PLANE and f.normal_at(f.center()).dot(nn) > 0.9999
    ]
    top = max(Vector(f.center()).dot(nn) for f in candidates)
    ref = max((f for f in candidates if Vector(f.center()).dot(nn) > top - 1e-6), key=lambda f: f.area)
    return ContactPatch(
        face=patches[0],
        center=center,
        area=area,
        normal=nn,
        support_area=ref.area,
        support_outline=Face(ref.outer_wire()),
    )


@dataclass(frozen=True)
class Frame:
    """Orthonormal basis for 2D calculations on a plane."""

    origin: Vector
    n: Vector
    u: Vector
    v: Vector

    @classmethod
    def from_normal(cls, origin: Vector, n: Vector, u_hint: Vector | None = None) -> Frame:
        n = n.normalized()
        cand = u_hint if u_hint is not None else (Vector(1, 0, 0) if abs(n.X) < 0.9 else Vector(0, 1, 0))
        u = cand - n * cand.dot(n)
        if u.length < 1e-9:
            cand = Vector(0, 1, 0) if abs(n.Y) < 0.9 else Vector(0, 0, 1)
            u = cand - n * cand.dot(n)
        u = u.normalized()
        return cls(origin, n, u, n.cross(u).normalized())

    def to_2d(self, p: Vector) -> tuple[float, float]:
        d = Vector(p) - self.origin
        return (d.dot(self.u), d.dot(self.v))

    def height(self, p: Vector) -> float:
        return (Vector(p) - self.origin).dot(self.n)

    def plane(self, height: float = 0.0) -> Plane:
        return Plane(origin=self.origin + self.n * height, x_dir=self.u, z_dir=self.n)


def triangle_min_height(a, b, c) -> float:
    """Shortest altitude of a 2D triangle (mm). Collinear points return 0."""
    area2 = abs((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]))
    longest = max(math.dist(a, b), math.dist(b, c), math.dist(c, a))
    return 0.0 if longest == 0 else area2 / longest


def polygon_edge_margin(p, vertices: list[tuple[float, float]]) -> float:
    """Signed distance from point `p` to the nearest edge of a convex polygon: + inside, - outside."""
    n = len(vertices)
    # determine winding order (CW / CCW)
    area2 = sum(
        vertices[i][0] * vertices[(i + 1) % n][1] - vertices[(i + 1) % n][0] * vertices[i][1]
        for i in range(n)
    )
    sign = 1.0 if area2 > 0 else -1.0
    smallest = math.inf
    for i in range(n):
        x1, y1 = vertices[i]
        x2, y2 = vertices[(i + 1) % n]
        edge_len = math.hypot(x2 - x1, y2 - y1)
        if edge_len == 0:
            continue
        # left side is positive (interior for a CCW polygon)
        d = sign * ((x2 - x1) * (p[1] - y1) - (y2 - y1) * (p[0] - x1)) / edge_len
        smallest = min(smallest, d)
    return smallest


def face_edge_margin(point: Vector, face: Face) -> float:
    """Signed distance from the projection of `point` onto `face`'s plane to its boundary: + inside, - outside."""
    n = face.normal_at(face.center())
    proj = Vector(point) - n * (Vector(point) - Vector(face.center())).dot(n)
    outside = face.distance_to(proj)
    if outside > 1e-6:
        return -outside
    return min(e.distance_to(proj) for e in face.edges())


def section_points(body: Shape, frame: Frame, height: float, samples: int = 96) -> list[tuple[float, float]]:
    """2D edge sample points of the body's cross-section perpendicular to the frame normal at `height`."""
    plane = frame.plane(height)
    section = body.intersect(plane.to_face(1e4, 1e4) if hasattr(plane, "to_face") else Face(plane, 1e4, 1e4))
    if not section:
        return []
    points = []
    for e in section.edges():
        for i in range(samples + 1):
            points.append(frame.to_2d(e.position_at(i / samples)))
    return points


def width(points: list[tuple[float, float]], angle_deg: float) -> float:
    """Span of a 2D point cloud along the given direction."""
    a = math.radians(angle_deg)
    d = (math.cos(a), math.sin(a))
    proj = [x * d[0] + y * d[1] for x, y in points]
    return max(proj) - min(proj)


def narrowest_direction(points: list[tuple[float, float]], step: float = 0.5) -> tuple[float, float, float]:
    """(angle_deg, min_width, max_width). Angle is measured in [0, 180) from the u axis."""
    angles = [i * step for i in range(int(180 / step))]
    widths = [(width(points, a), a) for a in angles]
    narrowest = min(widths)
    widest = max(widths)
    return narrowest[1], narrowest[0], widest[0]


def axis_angle_diff(a: float, b: float) -> float:
    """Angle between two undirected axes, in 0..90°."""
    diff = abs((a - b) % 180.0)
    return min(diff, 180.0 - diff)


def cylinder_diameters(body: Shape) -> list[tuple[float, Vector, Vector]]:
    """List of (diameter, axis_origin, axis_direction) for all cylindrical faces of `body`."""
    result = []
    for f in body.faces():
        if f.geom_type != GeomType.CYLINDER:
            continue
        ax = f.axis_of_rotation
        result.append((2.0 * f.radius, Vector(ax.position), Vector(ax.direction)))
    return result
