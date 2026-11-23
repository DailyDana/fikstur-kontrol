"""ISO 286 tolerance zones (hole and shaft) and explicit limits.

Scope is intentionally narrow: zones commonly used for locating pins and holes.
Holes: E, F, G, H, JS. Shafts: e, f, g, h, js, k, n, p. Grades: IT5..IT11. Diameter: 0..500 mm.
Values from ISO 286-1/-2 tables (µm); every zone is unit-tested against known values.
"""

from __future__ import annotations

from bisect import bisect_left
from dataclasses import dataclass

# Upper bounds of diameter steps: "over 3, up to and including 6" ... (first step 0..3 inclusive)
_RANGE_UPPER = [3, 6, 10, 18, 30, 50, 80, 120, 180, 250, 315, 400, 500]

# Standard tolerance grades (µm)
_IT = {
    5: [4, 5, 6, 8, 9, 11, 13, 15, 18, 20, 23, 25, 27],
    6: [6, 8, 9, 11, 13, 16, 19, 22, 25, 29, 32, 36, 40],
    7: [10, 12, 15, 18, 21, 25, 30, 35, 40, 46, 52, 57, 63],
    8: [14, 18, 22, 27, 33, 39, 46, 54, 63, 72, 81, 89, 97],
    9: [25, 30, 36, 43, 52, 62, 74, 87, 100, 115, 130, 140, 155],
    10: [40, 48, 58, 70, 84, 100, 120, 140, 160, 185, 210, 230, 250],
    11: [60, 75, 90, 110, 130, 160, 190, 220, 250, 290, 320, 360, 400],
}

# Shaft fundamental deviations (µm). Upper deviation es for e, f, g, h; lower deviation ei for k, n, p.
_SHAFT_ES = {
    "e": [-14, -20, -25, -32, -40, -50, -60, -72, -85, -100, -110, -125, -135],
    "f": [-6, -10, -13, -16, -20, -25, -30, -36, -43, -50, -56, -62, -68],
    "g": [-2, -4, -5, -6, -7, -9, -10, -12, -14, -15, -17, -18, -20],
    "h": [0] * 13,
}
_SHAFT_EI = {
    # k: only for IT4..IT7; for coarser grades ei = 0
    "k": [0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 5],
    "n": [4, 8, 10, 12, 15, 17, 20, 23, 27, 31, 34, 37, 40],
    "p": [6, 12, 15, 18, 22, 26, 32, 37, 43, 50, 56, 62, 68],
}


def _range_index(dia: float) -> int:
    if not 0 < dia <= _RANGE_UPPER[-1]:
        raise ValueError(f"diameter {dia} mm is outside ISO 286 range (0..500 mm]")
    return bisect_left(_RANGE_UPPER, dia)


def _it(grade: int, i: int) -> float:
    if grade not in _IT:
        raise ValueError(f"IT{grade} is not supported (IT5..IT11)")
    return _IT[grade][i] / 1000.0


def _parse_zone(zone: str) -> tuple[str, int]:
    letter = zone.rstrip("0123456789")
    digits = zone[len(letter):]
    if not letter or not digits:
        raise ValueError(f"invalid tolerance zone: {zone!r} (e.g. 'H7', 'e7', 'js6')")
    return letter, int(digits)


@dataclass(frozen=True)
class Tolerance:
    """Lower and upper limits of a dimension (mm). `name` is for reporting."""

    lower: float
    upper: float
    name: str = ""

    def __post_init__(self) -> None:
        if self.lower > self.upper:
            raise ValueError(f"lower limit exceeds upper limit: {self.lower} > {self.upper}")

    @property
    def mid(self) -> float:
        return 0.5 * (self.lower + self.upper)

    def contains(self, value: float, margin: float = 1e-6) -> bool:
        return self.lower - margin <= value <= self.upper + margin

    @classmethod
    def limits(cls, lower: float, upper: float, name: str = "") -> Tolerance:
        return cls(lower, upper, name or f"{lower:.3f}..{upper:.3f}")

    @classmethod
    def plus_minus(cls, nominal: float, deviation: float) -> Tolerance:
        return cls(nominal - deviation, nominal + deviation, f"{nominal:g} ±{deviation:g}")

    @classmethod
    def deviation(cls, nominal: float, lower: float, upper: float) -> Tolerance:
        """Dimension with explicit deviations: `Tolerance.deviation(9.0, 0.0, +0.10)` = Ø9 +0.10/0."""
        return cls(nominal + lower, nominal + upper, f"{nominal:g} {upper:+g}/{lower:+g}")

    @classmethod
    def iso(cls, nominal: float, zone: str) -> Tolerance:
        """ISO 286 zone. Uppercase for holes, lowercase for shafts: `Tolerance.iso(9, 'e7')`."""
        letter, grade = _parse_zone(zone)
        i = _range_index(nominal)
        it = _it(grade, i)
        name = f"Ø{nominal:g} {zone}"
        if letter in ("js", "JS"):
            # ISO 286-1: for IT7..IT11, odd µm values are rounded down to the nearest even number
            um = _IT[grade][i]
            if grade >= 7 and um % 2:
                um -= 1
            return cls(nominal - um / 2000, nominal + um / 2000, name)
        if letter.islower():
            if letter in _SHAFT_ES:
                es = _SHAFT_ES[letter][i] / 1000.0
                return cls(nominal + es - it, nominal + es, name)
            if letter in _SHAFT_EI:
                ei = _SHAFT_EI[letter][i] / 1000.0
                if letter == "k" and grade > 7:
                    ei = 0.0
                return cls(nominal + ei, nominal + ei + it, name)
            raise ValueError(f"shaft zone {letter!r} is not supported (e, f, g, h, js, k, n, p)")
        # Hole: for A..H, EI = -(es of the matching lowercase shaft)
        low = letter.lower()
        if low in _SHAFT_ES:
            ei = -_SHAFT_ES[low][i] / 1000.0
            return cls(nominal + ei, nominal + ei + it, name)
        raise ValueError(f"hole zone {letter!r} is not supported (E, F, G, H, JS)")


@dataclass(frozen=True)
class Fit:
    """Extreme clearance conditions of a hole-pin pair (diametral, mm)."""

    hole: Tolerance
    pin: Tolerance

    @property
    def min_clearance(self) -> float:
        """Worst case: smallest hole, largest pin. If negative, the part may not assemble."""
        return self.hole.lower - self.pin.upper

    @property
    def max_clearance(self) -> float:
        """Loosest case: largest hole, smallest pin. Upper bound on positional uncertainty."""
        return self.hole.upper - self.pin.lower
