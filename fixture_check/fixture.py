"""Fixture specification: declare body roles to build a cad-khana Assembly and assertions automatically.

    f = (Fixture.step("assembly.step", part="Bracket")
         .datum_a(["Pad_1", "Pad_2", "Pad_3"])
         .round_pin("Pin_B", hole=Tolerance.deviation(9, 0, 0.1), pin=Tolerance.iso(9, "e7"))
         .diamond_pin("Pin_C", hole=..., pin=...)
         .clamp("Clamp_Left_Pad", group="Clamp_Left*")
         .contour("Block_Left", gap=3.0)
         .flush("Block_Left", direction=(0, 1, 0))
         .removal(direction=(0, 0, 1), distance=65))
    assembly = f.assembly()

Check IDs (prefixes of assertion names in the report) are documented in README.md.
Assertions prefixed with `warn:` are unbounded records: their value is reported without
turning `khana check` red.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from fnmatch import fnmatchcase
from pathlib import Path

from build123d import Axis, CenterOf, Cylinder, GeomType, Part, Plane, Shape, Vector
from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.keepout import swept

from . import geometry as geo
from .tolerance import Fit, Tolerance

SECTION_MARGIN = 0.01  # mm, to avoid slicing exactly on the part surface


# --- role records --------------------------------------------------------------------------

@dataclass(frozen=True)
class _Pin:
    name: str
    diamond: bool
    hole: Tolerance | None
    pin: Tolerance | None
    max_shift: float | None
    min_shoulder: float
    angle_tol: float = 5.0
    min_relief: float = 0.5
    datum_ref: str = "MMC"
    max_pin_play: float | None = None


@dataclass(frozen=True)
class _Clamp:
    pad: str
    group: str | None
    pad_margin: float
    arm: tuple[str, ...] = ()
    pivot: tuple[tuple[float, float, float], tuple[float, float, float]] | None = None
    opening: float = 100.0
    step: float = 1.0
    against: str = "A"


@dataclass(frozen=True)
class _SideDatum:
    label: str                 # "B" (secondary, two rest buttons) or "C" (tertiary, one rest button)
    pads: tuple[str, ...]
    min_spacing: float = 0.0


@dataclass(frozen=True)
class _Contour:
    name: str
    gap: float
    tol: float
    uniformity: float | None


@dataclass(frozen=True)
class _Flush:
    name: str
    direction: tuple[float, float, float]
    tol: float
    max_lateral: float
    point: tuple[float, float, float] | None
    opposed: bool


@dataclass(frozen=True)
class _Gauge:
    name: str
    direction: tuple[float, float, float]
    retract: float
    stroke: float
    go_dia: float
    nogo_dia: float | None
    hole: Tolerance | None
    position_tol: float | None
    modifier: str
    gauge_tol: float
    bushing: str | None
    min_bushing_ld: float


@dataclass(frozen=True)
class _DatumA:
    pads: tuple[str, ...]
    n_2_1: bool
    min_height: float
    min_seating: float


# --- main class ----------------------------------------------------------------------------

@dataclass(frozen=True)
class Fixture:
    """Immutable fixture specification. Each method returns a new Fixture."""

    bodies: dict[str, Part]
    part: str
    _datum: _DatumA | None = None
    _datum_b: _SideDatum | None = None
    _datum_c: _SideDatum | None = None
    _pins: tuple[_Pin, ...] = ()
    _clamps: tuple[_Clamp, ...] = ()
    _contours: tuple[_Contour, ...] = ()
    _flushes: tuple[_Flush, ...] = ()
    _moving: tuple[str, ...] = ()
    _known: tuple[tuple[str, str, str], ...] = ()
    _removal: tuple[tuple[float, float, float], float, bool] | None = None
    _gauges: tuple[_Gauge, ...] = ()
    _base: str | None = None
    _accepted_floating: tuple[tuple[str, str], ...] = ()
    connection_tol: float = 0.05  # mm; a guided fit with clearance up to this still counts as connected
    path_step: float = 1.0  # mm; sampling step for removal/gauge paths when an exact sweep is not possible
    min_clearance: float = 0.1
    gravity: tuple[float, float, float] | None = None
    strict_stability: bool = False
    _added: tuple[str, ...] = field(default=())  # bodies added via add_body (for reporting)

    # -- constructors
    @classmethod
    def step(cls, path: str | Path, part: str, **kwargs) -> Fixture:
        return cls.from_bodies(geo.step_bodies(path), part, **kwargs)

    @classmethod
    def from_bodies(cls, bodies: dict[str, Shape], part: str, **kwargs) -> Fixture:
        g = {name: geo.to_part(s) for name, s in bodies.items()}
        if part not in g:
            raise KeyError(f"part {part!r} not found among bodies")
        return cls(bodies=g, part=part, **kwargs)

    # -- body editing (for deriving faulty variants)
    def add_body(self, name: str, shape: Shape) -> Fixture:
        if name in self.bodies:
            raise KeyError(f"{name!r} already exists; use replace_body to replace it")
        return replace(self, bodies={**self.bodies, name: geo.to_part(shape)}, _added=self._added + (name,))

    def replace_body(self, name: str, shape: Shape) -> Fixture:
        self._name(name)
        return replace(self, bodies={**self.bodies, name: geo.to_part(shape)})

    # -- roles
    def datum_a(self, pads, *, n_2_1: bool = False, min_height: float = 10.0,
                min_seating: float = 0.95) -> Fixture:
        pads = tuple(self._name(p) for p in pads)
        return replace(self, _datum=_DatumA(pads, n_2_1, min_height, min_seating))

    def datum_b(self, pads, *, min_spacing: float = 30.0) -> Fixture:
        """Planar secondary datum: exactly two side rest buttons touching the datum B face (2 DOF).
        Use instead of a round pin when B is a face. `min_spacing`: smallest distance between the two buttons."""
        pads = tuple(self._name(x) for x in pads)
        if len(pads) != 2:
            raise ValueError("datum_b takes exactly two rest buttons")
        return replace(self, _datum_b=_SideDatum("B", pads, min_spacing))

    def datum_c(self, pad) -> Fixture:
        """Planar tertiary datum: one side rest button touching the datum C face (1 DOF).
        Use instead of a diamond pin when C is a face."""
        return replace(self, _datum_c=_SideDatum("C", (self._name(pad),)))

    def round_pin(self, name, *, hole: Tolerance | None = None, pin: Tolerance | None = None,
                  max_shift: float | None = None, min_shoulder: float = 1.0, datum_ref: str = "MMC",
                  max_pin_play: float | None = None) -> Fixture:
        """`datum_ref`: how the drawing references this datum feature, "MMC" or "RFS". A fixed pin simulates an MMC
        datum; for RFS a warn: record shows the shift a fixed pin allows (an expanding pin would remove it).
        `max_pin_play`: limit on play with the hole at MMC (hole MMC - smallest pin), the shift the pin adds."""
        _check_datum_ref(datum_ref)
        return replace(self, _pins=self._pins + (
            _Pin(self._name(name), False, hole, pin, max_shift, min_shoulder, datum_ref=datum_ref,
                 max_pin_play=max_pin_play),))

    def diamond_pin(self, name, *, hole: Tolerance | None = None, pin: Tolerance | None = None,
                    max_shift: float | None = None, min_shoulder: float = 1.0,
                    angle_tol: float = 5.0, min_relief: float = 0.5, datum_ref: str = "MMC",
                    max_pin_play: float | None = None) -> Fixture:
        _check_datum_ref(datum_ref)
        return replace(self, _pins=self._pins + (
            _Pin(self._name(name), True, hole, pin, max_shift, min_shoulder, angle_tol, min_relief,
                 datum_ref, max_pin_play),))

    def clamp(self, pad, *, group: str | None = None, pad_margin: float = 0.0, arm=(),
              pivot=None, opening: float = 100.0, step: float = 1.0, against: str = "A") -> Fixture:
        if against not in ("A", "B", "C"):
            raise ValueError("against must be 'A', 'B' or 'C'")
        """Clamp pad. `group` (pattern) and `arm` (names) mark opening bodies as moving.
        If `pivot=((x, y, z), (dx, dy, dz))` is given, the pad and `arm` bodies are rotated
        around that axis by `opening` degrees (sign sets direction) in `step` increments;
        each step is checked for interference with the part and fixed bodies (K9)."""
        if group is not None:
            self._pattern(group)
        arm = tuple(self._name(x) for x in arm)
        if pivot is not None:
            pivot = (tuple(pivot[0]), tuple(pivot[1]))
            if opening == 0 or step <= 0:
                raise ValueError("opening must be non-zero and step must be positive")
            if not arm and group is None:
                raise ValueError("when pivot is given, rotating bodies must be declared via arm= or group=")
        c = _Clamp(self._name(pad), group, pad_margin, arm, pivot, float(opening), float(step), against)
        return replace(self, _clamps=self._clamps + (c,))

    def contour(self, name, *, gap: float, tol: float = 0.01, uniformity: float | None = None) -> Fixture:
        return replace(self, _contours=self._contours + (_Contour(self._name(name), gap, tol, uniformity),))

    def flush(self, name, *, direction, tol: float = 0.01, max_lateral: float = 10.0, point=None,
              opposed: bool = False) -> Fixture:
        """Check whether a block face perpendicular to `direction` is coplanar with an adjacent part face.
        If `point` is given, the part face nearest to that point is used; otherwise the face pair within
        `max_lateral` with the smallest plane offset is chosen. The block face faces `direction`; the part
        face faces the same direction (or the opposite direction when `opposed=True`, e.g. when a block face
        is coplanar with an outer part face but points the opposite way)."""
        f = _Flush(self._name(name), tuple(direction), tol, max_lateral, tuple(point) if point else None, opposed)
        return replace(self, _flushes=self._flushes + (f,))

    def moving(self, *patterns: str) -> Fixture:
        for pat in patterns:
            self._pattern(pat)
        return replace(self, _moving=self._moving + patterns)

    def known_interference(self, a: str, b: str, reason: str) -> Fixture:
        return replace(self, _known=self._known + ((self._name(a), self._name(b), reason),))

    def accept_floating(self, pattern: str, reason: str) -> Fixture:
        """Bodies known not to connect to the base (e.g. an unmodeled clamp base).
        Excluded from K8 and reported as `warn:`; raises if the pattern matches nothing."""
        self._pattern(pattern)
        return replace(self, _accepted_floating=self._accepted_floating + ((pattern, reason),))

    def base(self, name: str) -> Fixture:
        return replace(self, _base=self._name(name))

    def removal(self, *, direction=(0, 0, 1), distance: float, exit: bool = True) -> Fixture:
        """Part lifts `distance` mm along `direction` without touching fixed bodies. With `exit` it must then go on
        until it is past every fixture body (catches overhead brackets, gauge towers and the like)."""
        return replace(self, _removal=(tuple(direction), float(distance), bool(exit)))

    def go_nogo_gauge(self, name, *, direction, stroke: float, go_dia: float, retract: float = 0.0,
                      nogo_dia: float | None = None, hole: Tolerance | None = None,
                      position_tol: float | None = None, modifier: str = "MMC",
                      gauge_tol: float = 0.005, bushing: str | None = None,
                      min_bushing_ld: float = 1.5) -> Fixture:
        if modifier not in ("MMC", "RFS"):
            raise ValueError("modifier must be 'MMC' or 'RFS'")
        if hole is not None and modifier == "MMC" and position_tol is None:
            raise ValueError("position_tol is required for an MMC gauge (GO = hole MMC - position tolerance)")
        g = _Gauge(self._name(name), tuple(direction), retract, stroke, go_dia, nogo_dia, hole, position_tol,
                   modifier, gauge_tol, self._name(bushing) if bushing else None, min_bushing_ld)
        return replace(self, _gauges=self._gauges + (g,))

    # -- helpers
    def _name(self, name: str) -> str:
        if name not in self.bodies:
            similar = [a for a in self.bodies if name.lower() in a.lower()][:5]
            raise KeyError(f"body {name!r} not found" + (f"; similar: {similar}" if similar else ""))
        return name

    def _pattern(self, pattern: str) -> list[str]:
        matched = [a for a in self.bodies if fnmatchcase(a, pattern)]
        if not matched:
            raise KeyError(f"pattern {pattern!r} matched no bodies")
        return matched

    def _moving_names(self) -> set[str]:
        names: set[str] = set()
        for pat in self._moving:
            names.update(self._pattern(pat))
        for c in self._clamps:
            if c.group:
                names.update(self._pattern(c.group))
            names.add(c.pad)
            names.update(c.arm)
        for g in self._gauges:
            names.add(g.name)
        names.discard(self.part)
        return names

    def _check_role_conflicts(self) -> None:
        roles: dict[str, str] = {}

        def assign(name: str, role: str) -> None:
            if name in roles:
                if roles[name] != role:
                    raise ValueError(f"{name!r} assigned to two roles: {roles[name]} and {role}")
                raise ValueError(f"{name!r} declared twice as {role}")
            roles[name] = role

        assign(self.part, "part")
        for p in (self._datum.pads if self._datum else ()):
            assign(p, "datum pad")
        for d in (self._datum_b, self._datum_c):
            for p in (d.pads if d else ()):
                assign(p, f"datum {d.label} rest button")
        for p in self._pins:
            assign(p.name, "locating pin")
        for c in self._clamps:
            assign(c.pad, "clamp pad")

    # -- Assembly construction ---------------------------------------------------------------
    def assembly(self) -> Assembly:
        self._check_role_conflicts()
        P = self.part
        g = self.bodies
        part = g[P]
        others = [a for a in g if a != P]
        moving = self._moving_names()
        fixed = [a for a in others if a not in moving]

        a = Assembly()
        for name, body in g.items():
            a = a.with_part(name, body)

        # K1 part is a single solid, K2 part interferes with no body
        a = a.assert_solid_count(P)
        a = a.assert_no_interference_between([P], others)

        # K7 fixture bodies do not interfere with each other (except known overlaps, tracked separately)
        known = [(x, y, r) for x, y, r in self._known]
        a = a.assert_no_interference_within(others, known_overlaps=known)
        for x, y, r in known:
            a = a.assert_interference(x, y, reason=r, name=f"known_interference:{x}/{y}")

        # Expected contacts and pins; every remaining body must be clear
        contacting = set(self._datum.pads if self._datum else ()) | {c.pad for c in self._clamps}
        for d in (self._datum_b, self._datum_c):
            contacting |= set(d.pads if d else ())
        pins = {p.name for p in self._pins}
        for name in sorted(contacting):
            a = a.assert_tangent_contact(P, name, name=f"contact:{name}")
        contour_names = {k.name for k in self._contours}
        gauge_names = {m.name for m in self._gauges}
        for name in others:
            if name in contacting or name in pins or name in contour_names or name in gauge_names:
                continue
            a = a.assert_distance(P, name, min_mm=self.min_clearance, name=f"clear:{name}")

        frame, patches, pin_info = None, {}, {}
        if self._datum:
            a, frame, patches = self._datum_checks(a, part)
        if self._pins:
            if frame is None:
                raise ValueError("datum_a must be defined before pin checks")
            a, frame, pin_info = self._pin_checks(a, part, frame)
        side = {}
        if self._datum_b or self._datum_c:
            if frame is None:
                raise ValueError("datum_a must be defined before datum_b / datum_c")
            a, side = self._side_datum_checks(a, part, frame)
        if self._datum:
            a = self._dof_checks(a, patches, pin_info, side)
        if self._clamps:
            if not patches:
                a = a.assert_scalar("warn:F3:skipped", 0.0,
                                    detail="datum_a not defined or no pad contact; clamp-to-pad alignment skipped")
            else:
                a = self._clamp_checks(a, part, patches, frame, side)
        for k in self._contours:
            a = self._contour_check(a, part, k)
        for f in self._flushes:
            a = self._flush_check(a, part, f)
        a = self._connectivity(a, others)
        if self._removal:
            direction, distance, exit_ = self._removal
            d = Vector(*direction).normalized()
            v = d * distance
            a = _clear_path(a, fixed, part, v, name=f"removal_path:{distance:g}mm", step=self.path_step)
            if exit_:
                a = self._removal_exit(a, part, fixed, d, distance)
        for m in self._gauges:
            a = self._gauge_check(a, part, m)
        for c in self._clamps:
            if c.pivot is not None:
                a = self._clamp_opening(a, part, c, moving)
            else:
                a = a.assert_scalar(f"warn:K9:skipped:{c.pad}", 0.0,
                                    detail="no pivot provided; clamp opening arc not checked")
        return a

    def _removal_exit(self, a: Assembly, part: Part, fixed, d: Vector, distance: float) -> Assembly:
        def extent(shape):
            bb = shape.bounding_box()
            corners = [Vector(x, y, z) for x in (bb.min.X, bb.max.X) for y in (bb.min.Y, bb.max.Y)
                       for z in (bb.min.Z, bb.max.Z)]
            return min(c.dot(d) for c in corners), max(c.dot(d) for c in corners)
        part_lo, _ = extent(part)
        far = max(extent(self.bodies[n])[1] for n in fixed) if fixed else part_lo
        rest = far - part_lo - distance + 1.0
        if rest <= 0:
            return a.assert_scalar("removal_exit_mm", 0.0, le=0.0,
                                   detail=f"after {distance:g} mm the part is already past every fixed body")
        start = part.moved(_translation(d * distance))
        return _clear_path(a, fixed, start, d * rest, name="removal_exit", step=max(self.path_step, rest / 100.0))

    # -- F1, F2: datum A ---------------------------------------------------------------------
    def _datum_checks(self, a: Assembly, part: Part):
        d = self._datum
        patches = {}
        for pad in d.pads:
            y = geo.contact_patch(self.bodies[pad], part)
            if y is not None:
                patches[pad] = y
        a = a.assert_scalar("F1:pads_in_contact", float(len(patches)),
                            ge=3, le=None if d.n_2_1 else 3,
                            detail=f"declared {len(d.pads)} pads; in contact: {sorted(patches)}")
        if len(patches) < 3:
            # cannot establish a plane; remaining datum checks are meaningless
            return a, geo.Frame.from_normal(Vector(0, 0, 0), Vector(0, 0, 1)), patches

        normals = [y.normal for y in patches.values()]
        sum_n = sum(normals, Vector(0, 0, 0))
        if sum_n.length < 1e-6:
            a = a.assert_scalar("F1:pad_normals_inconsistent", 0.0, ge=1.0,
                                detail="pad contact normals cancel each other out")
            return a, geo.Frame.from_normal(Vector(0, 0, 0), Vector(0, 0, 1)), {}
        n = sum_n.normalized()
        deviation = max(math.degrees(math.acos(max(-1.0, min(1.0, x.dot(n))))) for x in normals)
        a = a.assert_scalar("F1:pad_normal_deviation_deg", deviation, le=0.1)
        origin = sum((y.center for y in patches.values()), Vector(0, 0, 0)) / len(patches)
        frame = geo.Frame.from_normal(origin, n)
        heights = [frame.height(y.center) for y in patches.values()]
        a = a.assert_scalar("F1:pad_height_diff_mm", max(heights) - min(heights), le=0.01)

        centers = [frame.to_2d(y.center) for y in patches.values()]
        hull = _convex_hull(centers)
        if len(hull) >= 3:
            min_h = (min(geo.triangle_min_height(hull[0], hull[i], hull[i + 1])
                         for i in range(1, len(hull) - 1)) if len(hull) == 3 else _hull_min_width(hull))
        else:
            min_h = 0.0
        a = a.assert_scalar("F1:support_triangle_min_height_mm", min_h, ge=d.min_height,
                            detail="if contact centers are collinear or too close, the part will rock")

        for pad, y in patches.items():
            a = a.assert_scalar(f"F1:pad_seating_ratio:{pad}", y.area / y.support_area, ge=d.min_seating,
                                detail="fraction of the pad top face touching the part (hole/bend overhang)")

        # F2 stability: projection of center of mass along gravity onto the support polygon
        gv = Vector(*self.gravity).normalized() if self.gravity else -n
        com = Vector(part.center(CenterOf.MASS))
        denom = gv.dot(n)
        if denom > -1e-6:
            margin = -math.inf  # gravity does not press the part onto the pads
        else:
            t = -frame.height(com) / denom
            proj = com + gv * t
            margin = geo.polygon_edge_margin(frame.to_2d(proj), hull) if len(hull) >= 3 else -math.inf
        name = "F2:stability_margin_mm" if self.strict_stability else "warn:F2:stability_margin_mm"
        a = a.assert_scalar(name, margin, ge=0.0 if self.strict_stability else None,
                            detail="distance from projected CoM to support polygon edge; negative = tips without clamps")
        return a, frame, patches

    # -- F3: clamp-over-pad alignment --------------------------------------------------------
    def _clamp_checks(self, a: Assembly, part: Part, patches: dict, frame, side: dict | None = None) -> Assembly:
        clamped: set[str] = set()
        for c in self._clamps:
            y = geo.contact_patch(self.bodies[c.pad], part)
            if y is None:
                continue  # contact assertion is already red
            if c.against != "A":
                a = self._pusher_check(a, c, y, (side or {}).get(c.against, {}))
                continue
            prefix = f"F3:clamp_over_pad_mm:{c.pad}"
            opposed = {pad: py for pad, py in patches.items() if y.normal.dot(py.normal) < -0.999}
            if not opposed:
                a = a.assert_scalar(prefix, -math.inf, ge=c.pad_margin,
                                    detail="clamp does not press opposite to the pads (side or bottom clamping)")
                continue
            margins = {pad: geo.face_edge_margin(y.center, py.support_outline) for pad, py in opposed.items()}
            best = max(margins, key=margins.get)
            a = a.assert_scalar(prefix, margins[best], ge=c.pad_margin,
                                detail=f"clamping center over nearest pad {best}; negative = pressing unsupported sheet")
            if margins[best] >= 0:
                clamped.add(best)
                filled = _column_fill_ratio(part, y.center, patches[best], frame)
                a = a.assert_scalar(f"F3:clamp_column_filled:{c.pad}", filled, ge=0.99,
                                    detail="column between clamp and pad must be solid part material (not an overhanging flange)")
        unclamped = sorted(set(patches) - clamped)
        return a.assert_scalar("warn:F3:unclamped_pads", float(len(unclamped)),
                               detail=f"pads without a clamp above them: {unclamped}" if unclamped else "every pad has a clamp")

    def _pusher_check(self, a: Assembly, c: _Clamp, y, buttons: dict) -> Assembly:
        name = f"F3:pusher_over_datum_{c.against}_mm:{c.pad}"
        if not buttons:
            return a.assert_scalar(name, -math.inf, ge=c.pad_margin,
                                   detail=f"datum {c.against} rest buttons not declared or not in contact")
        opposed = [b for b in buttons.values() if y.normal.dot(b.normal) < -0.999]
        if len(opposed) != len(buttons):
            return a.assert_scalar(name, -math.inf, ge=c.pad_margin,
                                   detail=f"pusher does not press opposite to the datum {c.against} buttons")
        radius = min(math.sqrt(b.area / math.pi) for b in opposed)
        n = opposed[0].normal
        if len(opposed) == 2:
            p1, p2 = opposed[0].center, opposed[1].center
            length = (p2 - p1).length
            u = (p2 - p1) / length
            d = y.center - p1
            t = d.dot(u)
            off = d - u * t
            off = off - n * off.dot(n)
            margin = min(t, length - t, radius - off.length)
            detail = (f"push line between the two buttons: {t:.1f} of {length:.1f} mm along, {off.length:.2f} mm off "
                      f"their line (button radius {radius:.1f}); outside = the push tilts the part off a button")
        else:
            d = y.center - opposed[0].center
            off = d - n * d.dot(n)
            margin = radius - off.length
            detail = f"push line {off.length:.2f} mm from the button centre (button radius {radius:.1f})"
        return a.assert_scalar(name, margin, ge=c.pad_margin, detail=detail)

    # -- K9: clamp opening arc ---------------------------------------------------------------
    def _clamp_opening(self, a: Assembly, part: Part, c: _Clamp, moving: set[str]) -> Assembly:
        axis = Axis(c.pivot[0], c.pivot[1])
        names = [c.pad, *c.arm]
        if c.group:
            names += [x for x in self._pattern(c.group) if x not in names]
        rotating = [(name, self.bodies[name]) for name in names]
        targets = [(self.part, part)] + [(name, g) for name, g in self.bodies.items()
                                         if name != self.part and name not in moving]
        bboxes = {name: g.bounding_box() for name, g in targets}
        sign = 1.0 if c.opening > 0 else -1.0
        steps = round(abs(c.opening) / c.step)
        hit = None
        for i in range(1, steps + 1):
            angle = sign * i * c.step
            for name_r, body_r in rotating:
                rotated = body_r.rotate(axis, angle)
                bb = rotated.bounding_box()
                for name_t, body_t in targets:
                    bt = bboxes[name_t]
                    if (bb.max.X < bt.min.X or bt.max.X < bb.min.X or bb.max.Y < bt.min.Y
                            or bt.max.Y < bb.min.Y or bb.max.Z < bt.min.Z or bt.max.Z < bb.min.Z):
                        continue
                    common = rotated.intersect(body_t)
                    v = sum(x.volume for x in common.solids()) if common else 0.0
                    if v > 1e-2:
                        hit = (abs(angle), name_r, name_t, v)
                        break
                if hit:
                    break
            if hit:
                break
        value = hit[0] if hit else abs(c.opening)
        detail = (f"at {hit[0]:g}° {hit[1]} → {hit[2]} ({hit[3]:.1f} mm³)" if hit
                  else f"clear through {abs(c.opening):g}° (sampled every {c.step:g}°)")
        return a.assert_scalar(f"K9:clamp_opening_deg:{c.pad}", value, ge=abs(c.opening), detail=detail)

    # -- F4, F5, G8: pins --------------------------------------------------------------------
    def _pin_checks(self, a: Assembly, part: Part, frame):
        info = {}
        for p in self._pins:
            info[p.name] = _measure_pin(self.bodies[p.name], part, frame)
        rounds = [p for p in self._pins if not p.diamond]
        diamonds = [p for p in self._pins if p.diamond]
        # rebuild frame aligned with the B→C line
        bc_spacing = None
        if (rounds and diamonds and not info[rounds[0].name].get("error")
                and not info[diamonds[0].name].get("error")):
            b = info[rounds[0].name]["center3"]
            c = info[diamonds[0].name]["center3"]
            hint = c - b
            bc_spacing = (hint - frame.n * hint.dot(frame.n)).length
            frame = geo.Frame.from_normal(frame.origin, frame.n, hint)
            info = {p.name: _measure_pin(self.bodies[p.name], part, frame) for p in self._pins}

        for p in self._pins:
            b = info[p.name]
            prefix = f"pin:{p.name}"
            if b is None or b.get("error"):
                a = a.assert_scalar(f"{prefix}:not_measurable", 0.0, ge=1.0,
                                    detail=(b or {}).get("error", "could not section pin in part thickness band"))
                continue
            a = a.assert_no_interference(self.part, p.name, name=f"{prefix}:no_interference")
            a = a.assert_scalar(f"{prefix}:full_engagement_dia_diff_mm", b["working_dia"] - b["min_dia_in_band"],
                                le=0.001, detail="cylindrical section of pin must span full part thickness; lead-in taper must not start inside the part")
            a = a.assert_scalar(f"{prefix}:cad_dia_clearance_mm", b["hole_dia"] - b["working_dia"], ge=0.0,
                                detail=f"CAD: hole Ø{b['hole_dia']:.3f}, pin Ø{b['working_dia']:.3f}")
            a = a.assert_scalar(f"{prefix}:shoulder_gap_mm", b["shoulder_gap"], ge=p.min_shoulder,
                                detail="pin shoulder must stay clear below the part (avoids over-constraint)")
            offset = b["offset_perp"] if p.diamond else b["offset"]
            if p.hole is not None and p.pin is not None:
                remaining = Fit(p.hole, p.pin).min_clearance - 2 * offset
                a = a.assert_scalar(f"{prefix}:worst_case_clearance_with_offset_mm", remaining, ge=0.0,
                                    detail=f"worst-case clearance - 2 x CAD axis offset ({offset:.4f} mm)")
            else:
                a = a.assert_scalar(f"{prefix}:hole_axis_offset_mm", offset, le=0.005,
                                    detail="distance between pin axis and hole axis (only component perpendicular to B→C for diamond)")
            if not p.diamond:
                a = a.assert_scalar(f"{prefix}:roundness", b["roundness"], ge=0.995,
                                    detail="min/max cross-section width; a pin declared round must actually be round")
            if p.pin is not None:
                a = a.assert_scalar(f"{prefix}:cad_dia_in_tolerance", b["working_dia"], ge=p.pin.lower, le=p.pin.upper,
                                    detail=f"CAD diameter must lie within {p.pin.name}")
            if p.hole is not None and p.pin is not None:
                fit = Fit(p.hole, p.pin)
                a = a.assert_scalar(f"{prefix}:worst_case_clearance_mm", fit.min_clearance, ge=0.0,
                                    detail=f"smallest hole {p.hole.lower:.3f} - largest pin {p.pin.upper:.3f}; negative = part may not assemble")
                name = f"{prefix}:datum_shift_mm" if p.max_shift is not None else f"warn:{prefix}:datum_shift_mm"
                a = a.assert_scalar(name, fit.max_clearance, le=p.max_shift,
                                    detail="diametral play available for the part to shift around the pin in the loosest condition")
                play = p.hole.lower - p.pin.lower
                name = f"{prefix}:pin_play_at_mmc_mm" if p.max_pin_play is not None else f"warn:{prefix}:pin_play_at_mmc_mm"
                a = a.assert_scalar(name, play, le=p.max_pin_play,
                                    detail=f"hole at MMC {p.hole.lower:.3f} - smallest pin {p.pin.lower:.3f}: shift the pin adds "
                                           f"beyond the hole tolerance ({p.hole.upper - p.hole.lower:.3f})")
                if p.datum_ref == "RFS":
                    a = a.assert_scalar(f"warn:{prefix}:rfs_datum_fixed_pin_shift_mm", fit.max_clearance,
                                        detail="the drawing references this datum RFS; a fixed pin lets the part shift by this "
                                               "much, an expanding (centering) pin would simulate the datum exactly")
            if p.diamond:
                a = a.assert_scalar(f"{prefix}:diamond_relief_mm", b["width_perp"] - b["width_bc"], ge=p.min_relief,
                                    detail=f"width along B→C {b['width_bc']:.3f}, perpendicular width {b['width_perp']:.3f}; a cylindrical pin is not a diamond pin")
                a = a.assert_scalar(f"{prefix}:diamond_orientation_error_deg", b["contact_dir_error"], le=p.angle_tol,
                                    detail="diamond pin's contacting (wide) direction must be perpendicular to B→C")
                if b.get("slot_angle") is not None:
                    a = a.assert_scalar(f"{prefix}:slot_orientation_error_deg", geo.axis_angle_diff(b["slot_angle"], 0.0),
                                        le=p.angle_tol, detail="long axis of the part slot must be parallel to B→C")
        if bc_spacing is not None:
            a = a.assert_scalar("warn:pin:B_C_spacing_mm", bc_spacing,
                                detail="distance between the two pins; short spacing weakens angular location")
        return a, frame, info

    # -- F6: degrees of freedom --------------------------------------------------------------
    def _dof_checks(self, a: Assembly, patches: dict, pin_info: dict, side: dict) -> Assembly:
        primary = min(3, len(patches))
        round_count = sum(1 for p in self._pins if not p.diamond)
        diamond_count = sum(1 for p in self._pins if p.diamond)
        b_dof = min(2, len(side.get("B", {}))) if self._datum_b else 0
        c_dof = min(1, len(side.get("C", {}))) if self._datum_c else 0
        total = primary + 2 * round_count + diamond_count + b_dof + c_dof
        detail = (f"primary {primary} + round pin {round_count}x2 + diamond pin {diamond_count}x1"
                  f" + datum B face {b_dof} + datum C face {c_dof} = {total}")
        a = a.assert_scalar("F6:constrained_dof", float(total), ge=6, le=6,
                            detail=detail + "; <6 under-constrained, >6 over-constrained")
        two = round_count + (1 if self._datum_b else 0)
        one = diamond_count + (1 if self._datum_c else 0)
        layout_ok = (two, one) == (1, 1)
        return a.assert_scalar("F6:pin_layout", 1.0 if layout_ok else 0.0, ge=1.0,
                               detail=("expected one 2-DOF locator (round pin or datum B face) and one 1-DOF locator "
                                       f"(diamond pin or datum C face); declared: {round_count} round pin, "
                                       f"{diamond_count} diamond pin, datum B face {bool(self._datum_b)}, "
                                       f"datum C face {bool(self._datum_c)}"))

    # -- F1B, F1C: planar secondary and tertiary datums ----------------------------------------
    def _side_datum_checks(self, a: Assembly, part: Part, frame):
        side = {}
        n_a = frame.n
        for d in (self._datum_b, self._datum_c):
            if d is None:
                continue
            got = {}
            for pad in d.pads:
                y = geo.contact_patch(self.bodies[pad], part)
                if y is not None:
                    got[pad] = y
            side[d.label] = got
            want = len(d.pads)
            a = a.assert_scalar(f"F1{d.label}:buttons_in_contact", float(len(got)), ge=want, le=want,
                                detail=f"datum {d.label} rest buttons in contact: {sorted(got)} of {list(d.pads)}")
            if len(got) < want:
                continue
            normals = [y.normal for y in got.values()]
            a = a.assert_scalar(f"F1{d.label}:perpendicular_to_A_error_deg", max(_perp_error(x, n_a) for x in normals),
                                le=0.1, detail=f"datum {d.label} face must be perpendicular to datum A")
            if d.label == "B":
                y1, y2 = list(got.values())
                a = a.assert_scalar("F1B:button_normal_deviation_deg", _angle(y1.normal, y2.normal), le=0.1,
                                    detail="both datum B buttons must touch the same plane")
                a = a.assert_scalar("F1B:button_offset_mm", abs((y2.center - y1.center).dot(y1.normal)), le=0.01,
                                    detail="the two buttons' contact planes must coincide")
                a = a.assert_scalar("F1B:button_spacing_mm", (y2.center - y1.center).length, ge=d.min_spacing,
                                    detail="close buttons give a short lever against rotation about the datum A normal")
            if d.label == "C" and side.get("B") and len(side["B"]) == 2:
                n_b = next(iter(side["B"].values())).normal
                a = a.assert_scalar("F1C:perpendicular_to_B_error_deg", _perp_error(normals[0], n_b), le=0.1,
                                    detail="datum C face must be perpendicular to datum B")
        return a, side


    # -- K4 gap ------------------------------------------------------------------------------
    def _contour_check(self, a: Assembly, part: Part, k: _Contour) -> Assembly:
        a = a.assert_distance(self.part, k.name, min_mm=k.gap - k.tol, max_mm=k.gap + k.tol, name=f"gap:{k.name}")
        block = self.bodies[k.name]
        distances = []
        for e in part.edges():
            if e.distance_to(block) > k.gap + 0.5:
                continue
            for i in range(17):
                d = block.distance_to(e.position_at(i / 16))
                if d <= k.gap + 0.5:
                    distances.append(d)
        if distances:
            spread = max(distances) - min(distances)
            name = f"gap_uniformity_mm:{k.name}" if k.uniformity is not None else f"warn:gap_uniformity_mm:{k.name}"
            a = a.assert_scalar(name, spread, le=k.uniformity,
                                detail=f"max-min on part edge samples within {k.gap + 0.5:g} mm of block ({len(distances)} points)")
        return a

    # -- flush -------------------------------------------------------------------------------
    def _flush_check(self, a: Assembly, part: Part, f: _Flush) -> Assembly:
        direction = Vector(*f.direction).normalized()
        block = self.bodies[f.name]
        sign = -1.0 if f.opposed else 1.0
        b_faces = [y for y in block.faces() if y.geom_type == GeomType.PLANE
                   and y.normal_at(y.center()).dot(direction) > 0.9999]
        p_faces = [y for y in part.faces() if y.geom_type == GeomType.PLANE
                   and sign * y.normal_at(y.center()).dot(direction) > 0.9999]
        if f.point is not None:
            target = Vector(*f.point)
            p_faces = sorted(p_faces, key=lambda y: y.distance_to(target))[:1]
        best = None
        for bf in b_faces:
            for pf in p_faces:
                diff = (Vector(bf.center()) - Vector(pf.center())).dot(direction)
                # lateral gap: translate the part face into the block plane and measure distance
                lateral = bf.distance_to(pf.translate(direction * diff))
                if f.point is None and lateral > f.max_lateral:
                    continue
                key = (abs(diff), lateral)
                if best is None or key < best[0]:
                    best = (key, diff, lateral)
        if best is None:
            return a.assert_scalar(f"flush:{f.name}:face_not_found", 0.0, ge=1.0,
                                   detail=f"no planar face pair perpendicular to {tuple(f.direction)}")
        return a.assert_scalar(f"flush:{f.name}:{_direction_name(f.direction)}", best[1], ge=-f.tol, le=f.tol,
                               detail=f"plane offset between block and part faces (direction {tuple(f.direction)}), lateral gap {best[2]:.3f} mm")

    # -- K8 connectivity ---------------------------------------------------------------------
    def _connectivity(self, a: Assembly, names: list[str]) -> Assembly:
        if not names:
            return a
        g = self.bodies
        root = self._base or max(names, key=lambda x: g[x].volume)
        bboxes = {x: g[x].bounding_box() for x in names}

        def close(x, y):
            bx, by = bboxes[x], bboxes[y]
            tol = self.connection_tol
            if (bx.max.X < by.min.X - tol or by.max.X < bx.min.X - tol or bx.max.Y < by.min.Y - tol
                    or by.max.Y < bx.min.Y - tol or bx.max.Z < by.min.Z - tol or by.max.Z < bx.min.Z - tol):
                return False
            return g[x].distance_to(g[y]) <= self.connection_tol

        reached = {root}
        stack = [root]
        while stack:
            x = stack.pop()
            for y in names:
                if y not in reached and close(x, y):
                    reached.add(y)
                    stack.append(y)
        floating = sorted(set(names) - reached)
        for pattern, reason in self._accepted_floating:
            accepted = [x for x in floating if fnmatchcase(x, pattern)]
            floating = [x for x in floating if x not in accepted]
            a = a.assert_scalar(f"warn:K8:accepted_floating:{pattern}", float(len(accepted)),
                                detail=f"{reason}; matched floating bodies: {accepted}")
        return a.assert_scalar("K8:floating_bodies", float(len(floating)), le=0,
                               detail=(f"not connected to {root} by contact: {floating}" if floating
                                       else f"all bodies connected to {root} by contact"))

    # -- K6, F7: GO/NO-GO gauge --------------------------------------------------------------
    def _gauge_check(self, a: Assembly, part: Part, m: _Gauge) -> Assembly:
        prefix = f"gauge:{m.name}"
        body = self.bodies[m.name]
        direction = Vector(*m.direction).normalized()
        cylinders = [(c, o, d) for c, o, d in geo.cylinder_diameters(body) if abs(d.dot(direction)) > 0.9999]
        if not cylinders:
            return a.assert_scalar(f"{prefix}:axis_not_found", 0.0, ge=1.0,
                                   detail="no cylindrical face along the gauge direction")
        axis_point = cylinders[0][1]
        start = body.moved(_translation(-direction * m.retract))
        others = [x for x in self.bodies if x != m.name]
        a = _clear_path(a, others, start, direction * m.stroke, name=f"{prefix}:path_clear", step=self.path_step)

        # GO cylinder must pass completely through the hole at the end of the stroke
        go_faces = [f for f in body.faces() if f.geom_type == GeomType.CYLINDER
                    and abs(2 * f.radius - m.go_dia) < 0.05
                    and abs(Vector(f.axis_of_rotation.direction).dot(direction)) > 0.9999]
        if go_faces:
            go_dia = 2 * go_faces[0].radius
            go_axis = Vector(go_faces[0].axis_of_rotation.position)
            proj = [Vector(v).dot(direction) for f in go_faces for v in f.vertices()]
            shift = m.stroke - m.retract
            go_start, go_tip = min(proj) + shift, max(proj) + shift
            # part hole: cylindrical face coaxial with GO (within clearance) and wider than GO
            hole_faces = [f for f in part.faces() if f.geom_type == GeomType.CYLINDER
                          and abs(Vector(f.axis_of_rotation.direction).dot(direction)) > 0.9999
                          and 2 * f.radius > go_dia
                          and _distance_to_axis(Vector(f.axis_of_rotation.position), go_axis, direction)
                          <= (2 * f.radius - go_dia) / 2 + 1e-6]
            if not hole_faces:
                a = a.assert_scalar(f"{prefix}:hole_not_found", 0.0, ge=1.0,
                                    detail="no hole in the part coaxial with GO that GO can pass through")
            else:
                hp = [Vector(v).dot(direction) for f in hole_faces for v in f.vertices()]
                h0, h1 = min(hp), max(hp)
                margin = min(go_tip - h1, h0 - go_start)
                a = a.assert_scalar(f"{prefix}:go_covers_hole_mm", margin, ge=0.0,
                                    detail=f"at end of stroke GO cylinder {go_start:.2f}..{go_tip:.2f}, "
                                           f"part hole Ø{2 * hole_faces[0].radius:.3f} {h0:.2f}..{h1:.2f} (along direction)")
        else:
            go_dia = None
            a = a.assert_scalar(f"{prefix}:go_cylinder_not_found", 0.0, ge=1.0, detail=f"no Ø{m.go_dia} cylinder found")

        nogo_dia = None
        if m.nogo_dia is not None:
            nf = [f for f in body.faces() if f.geom_type == GeomType.CYLINDER and abs(2 * f.radius - m.nogo_dia) < 0.05
                  and abs(Vector(f.axis_of_rotation.direction).dot(direction)) > 0.9999]
            nogo_dia = 2 * nf[0].radius if nf else None
            if nogo_dia is None:
                a = a.assert_scalar(f"{prefix}:nogo_cylinder_not_found", 0.0, ge=1.0,
                                    detail=f"no Ø{m.nogo_dia} (±0.05) cylinder on gauge axis")

        # F7 gauge diameters
        if m.hole is not None:
            if m.modifier == "MMC" and m.position_tol is not None and go_dia is not None:
                virtual = m.hole.lower - m.position_tol
                a = a.assert_scalar(f"{prefix}:F7:go_dia", go_dia, ge=virtual - 1e-6, le=virtual + m.gauge_tol,
                                    detail=f"GO = virtual condition {m.hole.lower:.3f} - {m.position_tol:.3f} = {virtual:.3f} (+{m.gauge_tol} gauge tolerance)")
            elif m.modifier == "RFS":
                a = a.assert_scalar(f"warn:{prefix}:F7:rfs_skipped", 0.0,
                                    detail="RFS position tolerance cannot be verified with a fixed functional gauge; F7 skipped")
            if nogo_dia is not None:
                a = a.assert_scalar(f"{prefix}:F7:nogo_dia", nogo_dia, ge=m.hole.upper - m.gauge_tol, le=m.hole.upper + 1e-6,
                                    detail=f"NO-GO = LMC {m.hole.upper:.3f} (-{m.gauge_tol} gauge tolerance)")

        # Bushing L/D
        if m.bushing:
            bc = [(c, o) for c, o, d in geo.cylinder_diameters(self.bodies[m.bushing])
                  if abs(d.dot(direction)) > 0.9999 and _distance_to_axis(o, axis_point, direction) < 0.05]
            if bc:
                inner = min(c for c, _ in bc)
                bore_faces = [f for f in self.bodies[m.bushing].faces() if f.geom_type == GeomType.CYLINDER
                              and abs(2 * f.radius - inner) < 1e-6]
                p = [Vector(v).dot(direction) for f in bore_faces for v in f.vertices()]
                length = max(p) - min(p)
                a = a.assert_scalar(f"{prefix}:bushing_L_D", length / inner, ge=m.min_bushing_ld,
                                    detail=f"bushing bore Ø{inner:.3f}, length {length:.1f} mm; short bushing binds the gauge")
            else:
                a = a.assert_scalar(f"{prefix}:bushing_coaxial", 0.0, ge=1.0,
                                    detail="no cylindrical bore in bushing coaxial with gauge (within 0.05 mm)")
        return a


# --- module-level helpers -------------------------------------------------------------------

def _angle(u: Vector, v: Vector) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, u.normalized().dot(v.normalized())))))


def _perp_error(u: Vector, v: Vector) -> float:
    """How far from 90 degrees the angle between u and v is."""
    return abs(90.0 - _angle(u, v))


def _check_datum_ref(datum_ref: str) -> None:
    if datum_ref not in ("MMC", "RFS"):
        raise ValueError("datum_ref must be 'MMC' or 'RFS'")


def _clear_path(a: Assembly, others, body, travel: Vector, *, name: str, step: float) -> Assembly:
    """assert_clear_of along a straight path: exact sweep when cad-khana can do it, otherwise stepped copies.

    cad-khana's swept() is exact only for planes, cylinders, cones and spheres. Real machined parts have fillets
    (torus faces) and freeform surfaces; for those the path is approximated by copies of the body every ``step`` mm,
    and a warn: record says so (an obstacle thinner than ``step`` between two copies could be missed).
    """
    try:
        path = swept(body, travel)
    except ValueError as exc:
        from math import ceil

        from build123d import Compound
        n = max(1, ceil(travel.length / step))
        copies = [body.moved(_translation(travel * (i / n))) for i in range(n + 1)]
        path = Compound(copies)
        kinds = str(exc).split("has ")[-1].split(" faces")[0]
        a = a.assert_scalar(f"warn:{name}:sampled_step_mm", travel.length / n,
                            detail=f"path sampled with {n + 1} copies (body has {kinds} faces; exact sweep not possible)")
    return a.assert_clear_of(others, path, name=name)


def _translation(v: Vector):
    from build123d import Location
    return Location((v.X, v.Y, v.Z))


def _distance_to_axis(p: Vector, axis_point: Vector, direction: Vector) -> float:
    d = Vector(p) - Vector(axis_point)
    return (d - direction * d.dot(direction)).length


def _axis_cylinder(point: Vector, direction: Vector, r: float, length: float = 2000.0) -> Part:
    plane = Plane(origin=Vector(point) - direction * (length / 2), z_dir=direction)
    from build123d import Align
    return geo.to_part(plane * Cylinder(r, length, align=(Align.CENTER, Align.CENTER, Align.MIN)))


def _column_fill_ratio(part: Part, clamp_center: Vector, pad, frame) -> float:
    """Fraction of a Ø2 column from the clamp pressure center down to the pad plane filled by part material."""
    from build123d import Align
    h = frame.height(clamp_center) - frame.height(pad.center)
    if h <= 1e-6:
        return 0.0
    base = Vector(clamp_center) - frame.n * h
    column = geo.to_part(Plane(origin=base, z_dir=frame.n) * Cylinder(1.0, h, align=(Align.CENTER, Align.CENTER, Align.MIN)))
    common = part.intersect(column)
    volume = sum(k.volume for k in common.solids()) if common else 0.0
    return volume / column.volume


def _convex_hull(points):
    """2D convex hull (counter-clockwise), monotone chain."""
    p = sorted(set(points))
    if len(p) <= 2:
        return p

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for x in p:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], x) <= 0:
            lower.pop()
        lower.append(x)
    for x in reversed(p):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], x) <= 0:
            upper.pop()
        upper.append(x)
    return lower[:-1] + upper[:-1]


def _hull_min_width(hull) -> float:
    """Minimum width of a convex polygon (rotating calipers, per edge)."""
    n = len(hull)
    narrowest = math.inf
    for i in range(n):
        x1, y1 = hull[i]
        x2, y2 = hull[(i + 1) % n]
        L = math.hypot(x2 - x1, y2 - y1)
        if L == 0:
            continue
        farthest = max(abs((x2 - x1) * (q[1] - y1) - (y2 - y1) * (q[0] - x1)) / L for q in hull)
        narrowest = min(narrowest, farthest)
    return narrowest


def _section_edges(body: Shape, frame, h: float):
    plane = frame.plane(h)
    from build123d import Face
    sec = body.intersect(Face(plane, 1e4, 1e4))
    return list(sec.edges()) if sec else []


def _measure_pin(pin: Part, part: Part, frame) -> dict | None:
    """Measure pin dimensions within the part's thickness band (in the frame's 2D plane)."""
    # rough pin axis point: closest point on the pin to the part
    _, p_pin, _ = pin.distance_to_with_closest_points(part)
    rough_center = Vector(p_pin)
    # part thickness band around the pin: first from the hole wall adjacent to the pin,
    # otherwise from a narrow cylinder intersection
    _, _, p_part = pin.distance_to_with_closest_points(part)
    walls = [f for f in part.faces() if f.geom_type == GeomType.CYLINDER
             and abs(Vector(f.axis_of_rotation.direction).dot(frame.n)) > 0.9999
             and f.distance_to(Vector(p_part)) < 1e-3]
    if walls:
        hs = [frame.height(v) for f in walls for v in f.vertices()]
    else:
        local = part.intersect(_axis_cylinder(rough_center, frame.n, 3.0))
        if not local or not local.vertices():
            return {"error": "no part material around the pin"}
        hs = [frame.height(v) for v in local.vertices()]
    h0, h1 = min(hs), max(hs)
    mid = 0.5 * (h0 + h1)

    def circles(h):
        return [e for e in _section_edges(pin, frame, h) if e.geom_type == GeomType.CIRCLE]

    c_mid = circles(mid)
    if not c_mid:
        return {"error": f"no circular edge in pin cross-section (height {mid:.2f})"}
    largest = max(c_mid, key=lambda e: e.radius)
    working = 2 * largest.radius
    center3 = Vector(largest.arc_center)
    band = [circles(h0 + SECTION_MARGIN), circles(h1 - SECTION_MARGIN)]
    min_in_band = min((2 * max(e.radius for e in c) if c else 0.0) for c in band)

    # hole: cylindrical faces on the part parallel and close to the pin axis
    candidates = []
    for c, o, d in geo.cylinder_diameters(part):
        if abs(d.dot(frame.n)) < 0.9999:
            continue
        dist = _distance_to_axis(o, center3, frame.n)
        if dist < working / 2 + 2.0:
            candidates.append((dist, c, o))
    if not candidates:
        return {"error": "no hole found in the part around the pin"}
    dist, hole_dia, hole_o = min(candidates, key=lambda x: (x[0], -x[1]))
    diff = Vector(center3) - Vector(hole_o)
    diff = diff - frame.n * diff.dot(frame.n)
    # slot: two half-cylinders of the same diameter at different centers -> long axis angle (from u axis)
    slot_angle = None
    same = [frame.to_2d(o) for _, c, o in candidates if abs(c - hole_dia) < 1e-3]
    for i in range(len(same)):
        for j in range(i + 1, len(same)):
            dx, dy = same[j][0] - same[i][0], same[j][1] - same[i][1]
            if math.hypot(dx, dy) > 0.05:
                slot_angle = math.degrees(math.atan2(dy, dx)) % 180

    # cross-section widths (for diamond pin)
    points = []
    for e in _section_edges(pin, frame, mid):
        points.extend(frame.to_2d(e.position_at(i / 96)) for i in range(97))
    width_bc = geo.width(points, 0.0)
    all_widths = [geo.width(points, i * 1.0) for i in range(180)]
    roundness = min(all_widths) / max(all_widths)
    width_perp = geo.width(points, 90.0)
    angles = [i * 0.5 for i in range(360)]
    widths = [geo.width(points, x) for x in angles]
    peak = max(widths)
    plateau = [x for x, w in zip(angles, widths) if w >= peak - 0.002]
    # axial mean (0..180 wrap-around): double angles and take the vector mean
    sx = sum(math.cos(math.radians(2 * x)) for x in plateau)
    sy = sum(math.sin(math.radians(2 * x)) for x in plateau)
    contact_dir = (math.degrees(math.atan2(sy, sx)) / 2) % 180
    contact_dir_error = geo.axis_angle_diff(contact_dir, 90.0)

    # shoulder: highest planar face on the pin below the part that is wider than the hole (over-constrains part)
    shoulder_h = -math.inf
    for f in pin.faces():
        if f.geom_type != GeomType.PLANE or abs(f.normal_at(f.center()).dot(frame.n)) < 0.9999:
            continue
        h = frame.height(f.center())
        if h > h0 + SECTION_MARGIN:
            continue
        outer_r = max(_distance_to_axis(Vector(e.position_at(t)), center3, frame.n)
                      for e in f.edges() for t in (0.0, 0.25, 0.5, 0.75))
        if outer_r > hole_dia / 2 + 1e-6:
            shoulder_h = max(shoulder_h, h)
    shoulder = max(0.0, h0 - shoulder_h)
    # tapered/chamfered shoulder: check whether the pin widens beyond the hole in sections just below the part
    for step in (SECTION_MARGIN, 0.25, 0.5, 1.0, 2.0):
        if step >= shoulder:
            break
        pts = [Vector(e.position_at(t)) for e in _section_edges(pin, frame, h0 - step) for t in (0.0, 0.25, 0.5, 0.75)]
        if pts and max(_distance_to_axis(q, center3, frame.n) for q in pts) > hole_dia / 2 + 1e-6:
            shoulder = step
            break

    return {
        "center3": center3,
        "working_dia": working,
        "min_dia_in_band": min_in_band,
        "hole_dia": hole_dia,
        "offset": diff.length,
        "offset_perp": abs(diff.dot(frame.v)),
        "width_bc": width_bc,
        "width_perp": width_perp,
        "contact_dir_error": contact_dir_error,
        "shoulder_gap": shoulder,
        "roundness": roundness,
        "slot_angle": slot_angle,
        "band": (h0, h1),
    }


def _direction_name(direction) -> str:
    """(0, 1, 0) -> '+Y'; numeric tuple if not axis-aligned."""
    names = {(1, 0, 0): "+X", (-1, 0, 0): "-X", (0, 1, 0): "+Y", (0, -1, 0): "-Y", (0, 0, 1): "+Z", (0, 0, -1): "-Z"}
    return names.get(tuple(round(float(x)) if float(x).is_integer() else x for x in direction),
                     "(" + ",".join(f"{float(x):g}" for x in direction) + ")")
