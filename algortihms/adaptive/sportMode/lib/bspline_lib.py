"""
B-spline library: Cox-de Boor basis and knot vector generation.
"""
import math
from typing import List, Tuple


def _make_knot_vector(n_ctrl: int, degree: int, clamped: bool = True) -> List[float]:
    """Create a uniform knot vector for n_ctrl control points.
    
    Args:
        n_ctrl: Number of control points.
        degree: Spline degree (typically 3 for cubic).
        clamped: If True, create open-uniform (clamped) knot vector.
    
    Returns:
        Normalized knot vector in [0, 1].
    """
    if n_ctrl <= degree:
        return [0.0] * (n_ctrl + degree + 1)

    m = n_ctrl + degree + 1
    if clamped:
        # Open uniform: first degree+1 zeros, last degree+1 ones, interior uniform
        interior_count = m - 2 * (degree + 1)
        knots = [0.0] * (degree + 1)
        if interior_count > 0:
            for i in range(1, interior_count + 1):
                knots.append(i / float(interior_count + 1))
        knots += [1.0] * (degree + 1)
    else:
        # Non-clamped uniform from 0..1
        knots = [i / float(m - 1) for i in range(m)]

    return knots


def _cox_de_boor(u: float, i: int, k: int, knots: List[float]) -> float:
    """Cox-de Boor recursion for B-spline basis function N_{i,k}(u).
    
    Args:
        u: Parameter value.
        i: Basis function index.
        k: Spline degree.
        knots: Knot vector.
    
    Returns:
        Basis function value N_{i,k}(u).
    """
    # Base case degree 0
    if k == 0:
        if knots[i] <= u < knots[i + 1]:
            return 1.0
        # Include last endpoint
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


def bspline_curve(ctrl_pts: List[Tuple[float, float]], degree: int = 3, 
                   samples: int = 200, clamped: bool = True) -> List[Tuple[float, float]]:
    """Sample a B-spline curve defined by control points.
    
    Args:
        ctrl_pts: List of control points (x, y).
        degree: Spline degree (cubic by default).
        samples: Number of sample points along parameter domain [0, 1].
        clamped: If True, use open-uniform knot vector.
    
    Returns:
        List of sampled (x, y) points.
    """
    n = len(ctrl_pts)
    if n == 0:
        return []
    if n == 1:
        return [(float(ctrl_pts[0][0]), float(ctrl_pts[0][1]))]

    # If too few control points for degree, lower degree
    deg = min(degree, n - 1)
    knots = _make_knot_vector(n, deg, clamped=clamped)

    out: List[Tuple[float, float]] = []
    if samples <= 0:
        samples = 200

    u_start = knots[deg]
    u_end = knots[-deg - 1]
    if u_end < u_start:
        u_start, u_end = knots[0], knots[-1]

    for si in range(samples + 1):
        u = u_start + (u_end - u_start) * (si / float(samples))
        # Ensure u within domain
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
