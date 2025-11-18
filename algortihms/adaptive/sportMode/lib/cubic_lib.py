"""
Natural cubic spline library: chord-length parameterization and Thomas tridiagonal solver.
"""
import math
from typing import List, Tuple


def _solve_tridiagonal(a: List[float], b: List[float], c: List[float], 
                       d: List[float]) -> List[float]:
    """Solve tridiagonal system Ax = d using Thomas algorithm.
    
    Args:
        a: sub-diagonal (a[0] unused)
        b: diagonal
        c: super-diagonal (c[-1] unused)
        d: right-hand side
    
    Returns:
        Solution vector x.
    """
    n = len(d)
    if n == 0:
        return []
    # Copy arrays to avoid modifying inputs
    cp = c.copy()
    dp = d.copy()
    bp = b.copy()

    # Forward sweep
    for i in range(1, n):
        if bp[i - 1] == 0.0:
            raise ZeroDivisionError("Zero diagonal on tridiagonal solve")
        m = a[i] / bp[i - 1]
        bp[i] = bp[i] - m * cp[i - 1]
        dp[i] = dp[i] - m * dp[i - 1]

    # Back substitution
    x = [0.0] * n
    if bp[-1] == 0.0:
        raise ZeroDivisionError("Zero diagonal on tridiagonal solve")
    x[-1] = dp[-1] / bp[-1]
    for i in range(n - 2, -1, -1):
        x[i] = (dp[i] - cp[i] * x[i + 1]) / bp[i]
    return x


def _natural_cubic_second_derivatives(t: List[float], y: List[float]) -> List[float]:
    """Compute second derivatives (M) for natural cubic spline on (t, y).
    
    Returns list M of length n with M[0] = M[-1] = 0 (natural boundary condition).
    """
    n = len(t)
    if n <= 2:
        return [0.0] * n

    h = [t[i + 1] - t[i] for i in range(n - 1)]
    # Build tridiagonal system for interior points 1..n-2
    a = [0.0] * (n - 2)
    b = [0.0] * (n - 2)
    c = [0.0] * (n - 2)
    d = [0.0] * (n - 2)
    for i in range(1, n - 1):
        idx = i - 1
        a[idx] = h[i - 1]
        b[idx] = 2.0 * (h[i - 1] + h[i])
        c[idx] = h[i]
        d[idx] = 6.0 * ((y[i + 1] - y[i]) / h[i] - (y[i] - y[i - 1]) / h[i - 1])

    # Solve for interior second derivatives
    M_interior = _solve_tridiagonal(a, b, c, d)

    M = [0.0] * n
    for i in range(1, n - 1):
        M[i] = M_interior[i - 1]
    return M


def _evaluate_spline_on_segment(ti: float, ti1: float, yi: float, yi1: float, 
                                 Mi: float, Mi1: float, t: float) -> float:
    """Evaluate natural cubic spline segment between ti and ti1 at parameter t."""
    h = ti1 - ti
    if h == 0:
        return yi
    A = (ti1 - t) / h
    B = (t - ti) / h
    term1 = (Mi * (A ** 3) * (h * h)) / 6.0
    term2 = (Mi1 * (B ** 3) * (h * h)) / 6.0
    term3 = (yi - (Mi * h * h) / 6.0) * A
    term4 = (yi1 - (Mi1 * h * h) / 6.0) * B
    return term1 + term2 + term3 + term4


def cubic_spline(path: List[Tuple[float, float]], 
                  samples_per_segment: int = 20) -> List[Tuple[float, float]]:
    """Sample a path through natural cubic spline using chord-length parameterization.
    
    Args:
        path: List of waypoints (x, y) to smooth.
        samples_per_segment: Number of samples per segment.
    
    Returns:
        List of sampled points along the cubic spline.
    """
    if not path:
        return []
    n = len(path)
    if n == 1:
        return [(float(path[0][0]), float(path[0][1]))]

    # Parameterize by chord length
    t: List[float] = [0.0]
    for i in range(1, n):
        dx = path[i][0] - path[i - 1][0]
        dy = path[i][1] - path[i - 1][1]
        t.append(t[-1] + math.hypot(dx, dy))

    # Handle degenerate case where all points coincide
    if t[-1] == 0.0:
        return [(float(path[0][0]), float(path[0][1])) for _ in range(samples_per_segment + 1)]

    xs = [float(p[0]) for p in path]
    ys = [float(p[1]) for p in path]

    Mx = _natural_cubic_second_derivatives(t, xs)
    My = _natural_cubic_second_derivatives(t, ys)

    out: List[Tuple[float, float]] = []
    for i in range(n - 1):
        ti = t[i]
        ti1 = t[i + 1]
        for k in range(samples_per_segment):
            u = k / float(samples_per_segment)
            tt = ti + u * (ti1 - ti)
            x = _evaluate_spline_on_segment(ti, ti1, xs[i], xs[i + 1], Mx[i], Mx[i + 1], tt)
            y = _evaluate_spline_on_segment(ti, ti1, ys[i], ys[i + 1], My[i], My[i + 1], tt)
            out.append((x, y))
    # Append final point exactly
    out.append((xs[-1], ys[-1]))
    return out
