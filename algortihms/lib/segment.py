"""Segment module for adaptive spline selection."""
import math
from typing import Tuple, List, Dict, Optional
from enum import Enum


class SelectionMode(Enum):
    """Selection modes for adaptive spline choice."""
    SPORT = "sport"
    EFFICIENT = "efficient"
    NORMAL = "normal"


class PathSegment:
    """Represents a single segment in a path."""
    
    def __init__(self, start: Tuple[float, float], end: Tuple[float, float], segment_id: int):
        self.start = start
        self.end = end
        self.segment_id = segment_id
        self.length = math.hypot(end[0] - start[0], end[1] - start[1])
    
    def __repr__(self) -> str:
        return f"Segment({self.segment_id}, {self.start} -> {self.end}, L={self.length:.2f})"


def create_segments(path: List[Tuple[float, float]]) -> List[PathSegment]:
    """Convert a path to segments."""
    segments = []
    for i in range(len(path) - 1):
        seg = PathSegment(path[i], path[i + 1], i)
        segments.append(seg)
    return segments


def compute_curvature_for_segment(prev_seg: Optional[PathSegment], curr_seg: Optional[PathSegment]) -> float:
    """Compute curvature proxy between two segments."""
    if prev_seg is None or curr_seg is None:
        return 0.0
    
    def heading(seg: PathSegment) -> float:
        dx = seg.end[0] - seg.start[0]
        dy = seg.end[1] - seg.start[1]
        return math.atan2(dy, dx)
    
    h1 = heading(prev_seg)
    h2 = heading(curr_seg)
    
    dh = abs(h2 - h1)
    if dh > math.pi:
        dh = 2.0 * math.pi - dh
    
    avg_len = (prev_seg.length + curr_seg.length) / 2.0
    if avg_len == 0.0:
        return 0.0
    
    return dh / avg_len


class AdaptiveSelector:
    """Selects the best spline based on cost metric."""
    
    def __init__(self, mode: SelectionMode = SelectionMode.SPORT):
        self.mode = mode
    
    def compute_cost(self, spline_length: float, curvature: float, gamma: float = 1.0) -> float:
        """Compute cost for a spline candidate."""
        return spline_length + gamma * curvature
    
    def select_best(self, candidates: Dict[str, Tuple[float, float]], gamma: float = 1.0) -> Tuple[str, float]:
        """Select the best spline from candidates."""
        best_name = None
        best_cost = float('inf')
        
        for name, (spline_len, curv) in candidates.items():
            cost = self.compute_cost(spline_len, curv, gamma)
            if cost < best_cost:
                best_cost = cost
                best_name = name
        
        return (best_name, best_cost) if best_name else ("", float('inf'))


def segment_length(segments: List[PathSegment]) -> float:
    """Compute total length of segments."""
    return sum(seg.length for seg in segments)
