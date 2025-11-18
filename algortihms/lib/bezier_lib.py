"""Bezier spline library."""
import math
from typing import Tuple, List


def _cubic_bezier_point(p0: Tuple[float, float], c1: Tuple[float, float], 
                         c2: Tuple[float, float], p3: Tuple[float, float], 
                         t: float) -> Tuple[float, float]:
    """Evaluate a cubic Bezier curve at parameter t in [0,1]."""
    u = 1.0 - t
    uu = u * u
    uuu = uu * u
    tt = t * t
    ttt = tt * t
    x = uuu * p0[0] + 3 * uu * t * c1[0] + 3 * u * tt * c2[0] + ttt * p3[0]
    y = uuu * p0[1] + 3 * uu * t * c1[1] + 3 * u * tt * c2[1] + ttt * p3[1]
    return (x, y)


def _catmull_rom_to_bezier_segments(points: List[Tuple[float, float]]) -> List[Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float], Tuple[float, float]]]:
    """Convert a polyline to cubic Bezier segments using Catmull-Rom conversion."""
    n = len(points)
    if n < 2:
        return []
    segments = []
    for i in range(n - 1):
        p0 = points[i - 1] if i - 1 >= 0 else points[i]
        p1 = points[i]
        p2 = points[i + 1]
        p3 = points[i + 2] if i + 2 < n else points[i + 1]

        c1 = (p1[0] + (p2[0] - p0[0]) / 6.0, p1[1] + (p2[1] - p0[1]) / 6.0)
        c2 = (p2[0] - (p3[0] - p1[0]) / 6.0, p2[1] - (p3[1] - p1[1]) / 6.0)
        segments.append((p1, c1, c2, p2))

    return segments


def bezier_spline(path: List[Tuple[float, float]], samples_per_segment: int = 20) -> List[Tuple[float, float]]:
    """Sample a path through cubic Bezier curve using Catmull-Rom conversion."""
    if not path:
        return []
    if len(path) < 2:
        return [(float(path[0][0]), float(path[0][1]))]

    segments = _catmull_rom_to_bezier_segments(path)
    out = []
    for (p0, c1, c2, p3) in segments:
        for i in range(samples_per_segment):
            t = i / float(samples_per_segment)
            out.append(_cubic_bezier_point(p0, c1, c2, p3, t))
    out.append((float(path[-1][0]), float(path[-1][1])))
    return out
