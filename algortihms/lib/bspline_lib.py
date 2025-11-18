"""B-spline library."""
import math
from typing import List, Tuple


def _make_knot_vector(n_ctrl: int, degree: int, clamped: bool = True) -> List[float]:
    """Create a uniform knot vector."""
    if n_ctrl <= degree:
        return [0.0] * (n_ctrl + degree + 1)

    m = n_ctrl + degree + 1
    if clamped:
        interior_count = m - 2 * (degree + 1)
        knots = [0.0] * (degree + 1)
        if interior_count > 0:
            for i in range(1, interior_count + 1):
                knots.append(i / float(interior_count + 1))
        knots += [1.0] * (degree + 1)
    else:
        knots = [i / float(m - 1) for i in range(m)]

    return knots


def _cox_de_boor(u: float, i: int, k: int, knots: List[float]) -> float:
    """Cox-de Boor recursion for B-spline basis function."""
    if k == 0:
        if knots[i] <= u < knots[i + 1]:
            return 1.0
        if u == knots[-1] and knots[i + 1] == knots[-1] and knots[i] < knots[i + 1]:
            return 1.0
        return 0.0

    denom1 = knots[i + k] - knots[i]
    term1 = 0.0
    if denom1 != 0.0:
        term1 = ((u - knots[i]) / denom1) * _cox_de_boor(u, i, k - 1, knots)

    denom2 = knots[i + k + 1] - knots[i + 1]
    term2 = 0.0
    if denom2 != 0.0:
        term2 = ((knots[i + k + 1] - u) / denom2) * _cox_de_boor(u, i + 1, k - 1, knots)

    return term1 + term2


def bspline_curve(ctrl_pts: List[Tuple[float, float]], degree: int = 3, samples: int = 200, clamped: bool = True) -> List[Tuple[float, float]]:
    """Sample a B-spline curve."""
    n = len(ctrl_pts)
    if n == 0:
        return []
    if n == 1:
        return [(float(ctrl_pts[0][0]), float(ctrl_pts[0][1]))]

    deg = min(degree, n - 1)
    knots = _make_knot_vector(n, deg, clamped=clamped)

    out = []
    if samples <= 0:
        samples = 200

    u_start = knots[deg]
    u_end = knots[-deg - 1]
    if u_end < u_start:
        u_start, u_end = knots[0], knots[-1]

    for si in range(samples + 1):
        u = u_start + (u_end - u_start) * (si / float(samples))
        if u < knots[0]:
            u = knots[0]
        if u > knots[-1]:
            u = knots[-1]

        x = 0.0
        y = 0.0
        for i in range(n):
            Ni = _cox_de_boor(u, i, deg, knots)
            x += Ni * float(ctrl_pts[i][0])
            y += Ni * float(ctrl_pts[i][1])
        out.append((x, y))

    return out
