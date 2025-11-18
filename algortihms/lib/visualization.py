"""Visualization module."""
import matplotlib.pyplot as plt
from typing import Set, List, Tuple, Optional, Dict
import math


def euclidean(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    """Compute Euclidean distance."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def plot_raw_path(width: int, height: int, obstacles: Set[Tuple[int, int]], path: List[Tuple[float, float]], fname: str):
    """Plot raw path."""
    fig, ax = plt.subplots(figsize=(8, 4))
    for x in range(width + 1):
        ax.plot([x, x], [0, height], color='lightgray', linewidth=0.5)
    for y in range(height + 1):
        ax.plot([0, width], [y, y], color='lightgray', linewidth=0.5)

    ox = [o[0] for o in obstacles]
    oy = [o[1] for o in obstacles]
    ax.scatter(ox, oy, marker='s', s=200, color='red', label='obstacle')

    if path:
        px = [p[0] for p in path]
        py = [p[1] for p in path]
        ax.plot(px, py, '-o', color='blue', linewidth=2, markersize=6, label='path')

    ax.set_xlim(-0.5, width + 0.5)
    ax.set_ylim(-0.5, height + 0.5)
    ax.set_aspect('equal')
    ax.set_title('Raw Path')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.legend()
    plt.grid(False)
    plt.savefig(fname, dpi=200, bbox_inches='tight')
    plt.close(fig)


def plot_spline_comparison(width: int, height: int, obstacles: Set[Tuple[int, int]], path: List[Tuple[float, float]], spline_dict: Dict[str, List[Tuple[float, float]]], length_dict: Dict[str, float], fname: str):
    """Plot spline comparison."""
    fig, ax = plt.subplots(figsize=(12, 6))
    
    for x in range(width + 1):
        ax.plot([x, x], [0, height], color='lightgray', linewidth=0.5)
    for y in range(height + 1):
        ax.plot([0, width], [y, y], color='lightgray', linewidth=0.5)

    ox = [o[0] for o in obstacles]
    oy = [o[1] for o in obstacles]
    ax.scatter(ox, oy, marker='s', s=200, color='red', label='obstacle', zorder=5)

    if path:
        px = [p[0] for p in path]
        py = [p[1] for p in path]
        ax.plot(px, py, '-o', color='blue', linewidth=2, markersize=5, label='raw path', zorder=3)

    colors = {'bezier': 'orange', 'cubic': 'green', 'bspline': 'purple', 'adaptive': 'red'}
    
    for name, pts in spline_dict.items():
        if pts:
            sx = [p[0] for p in pts]
            sy = [p[1] for p in pts]
            color = colors.get(name, 'gray')
            length_val = length_dict.get(name, 0.0)
            label = f'{name} (L={length_val:.2f})'
            ax.plot(sx, sy, '-', color=color, linewidth=2, label=label, zorder=2)

    ax.set_xlim(-0.5, width + 0.5)
    ax.set_ylim(-0.5, height + 0.5)
    ax.set_aspect('equal')
    ax.set_title('Spline Comparison')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.legend(loc='best')
    plt.grid(False)
    plt.savefig(fname, dpi=200, bbox_inches='tight')
    plt.close(fig)


def print_path_statistics(name: str, length: float, num_points: int):
    """Print path statistics."""
    print(f"{name:20s}: Length={length:8.3f}  Points={num_points:5d}")
