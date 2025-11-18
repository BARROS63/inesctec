# Adaptive Spline Selection Algorithm

## Overview

This project implements a modular architecture for comparing and adaptively selecting spline smoothing methods for 2D path planning. The algorithm uses **Lazy Theta\*** for pathfinding and evaluates three different spline types to find the optimal smoothing for each path segment.

## Architecture

### Modular Library Structure

The code is organized into independent library modules in the `lib/` directory:

#### Core Modules

- **`pathfinding.py`** - Lazy Theta\* pathfinding algorithm
  - `LazyThetaStar` class: A\* variant with line-of-sight shortcuts
  - `euclidean()`: Distance metric
  - `bresenham_line()`: Discrete line-of-sight checking
  - `path_length()`: Computes total path length

- **`bezier_lib.py`** - Cubic Bezier spline smoothing
  - `_cubic_bezier_point()`: Evaluates a cubic Bezier curve
  - `_catmull_rom_to_bezier_segments()`: Converts polyline to Bezier segments
  - `bezier_spline()`: Main sampler function

- **`cubic_lib.py`** - Natural cubic spline smoothing
  - `_solve_tridiagonal()`: Thomas algorithm for tridiagonal systems
  - `_natural_cubic_second_derivatives()`: Computes spline derivatives
  - `_evaluate_spline_on_segment()`: Segment evaluation
  - `cubic_spline()`: Main sampler function

- **`bspline_lib.py`** - B-spline smoothing
  - `_make_knot_vector()`: Generates knot vectors
  - `_cox_de_boor()`: Cox-de Boor basis recursion
  - `bspline_curve()`: Main sampler function

- **`segment.py`** - Path segments and adaptive selection framework
  - `PathSegment` class: Represents single edge between two nodes
  - `SelectionMode` enum: `SPORT` (shortest), `EFFICIENT` (low energy), `NORMAL` (balanced)
  - `AdaptiveSelector` class: Selects best spline per segment
  - `create_segments()`: Converts path to segments
  - `compute_curvature_for_segment()`: Curvature metric

- **`visualization.py`** - Plotting utilities
  - `plot_raw_path()`: Grid + obstacles + raw path
  - `plot_spline_comparison()`: Overlay comparison of all splines
  - `print_path_statistics()`: Formatted console output

### Main Integration

- **`main.py`** - Demonstrates the complete workflow:
  1. Sets up a 2D grid with obstacles
  2. Computes the shortest path (Lazy Theta\*)
  3. Applies three spline smoothing methods
  4. Performs adaptive selection (Sport mode = minimize path length)
  5. Generates comparison images and prints statistics

## Key Concepts

### Segments
- A **segment** is defined as a single edge between two consecutive path nodes
- The path is divided into segments for localized optimization
- Future modes can optimize per-segment based on different metrics

### Selection Modes
- **SPORT** (current): Minimize path length (shortest physical distance)
- **EFFICIENT** (future): Minimize energy loss (for UAV efficiency)
- **NORMAL** (future): Balanced trade-off between length and energy

### Cost Function
For each segment group, the cost is computed as:
```
cost = length + gamma * curvature
```
where:
- `length`: Total path length of the spline
- `curvature`: Heading change rate (normalized)
- `gamma`: Weighting factor (default 0.5)

## Usage

### Running the Demo

```bash
cd /path/to/algortihms
python3 main.py
```

This will:
- Generate two output PNG images in `outputs/`:
  - `raw_path.png`: The raw Lazy Theta* path
  - `adaptive_spline_comparison.png`: Overlay of all three splines with path lengths
- Print path length statistics and adaptive selection details to console

### Using Individual Libraries

```python
from lib.pathfinding import LazyThetaStar, path_length
from lib.bezier_lib import bezier_spline
from lib.cubic_lib import cubic_spline
from lib.bspline_lib import bspline_curve

# Compute path using Lazy Theta*
lts = LazyThetaStar(width=8, height=4, obstacles={(2,1), (5,2)})
path = lts.compute_path((0,0), (8,4))

# Apply spline smoothing
bezier_pts = bezier_spline(path, samples_per_segment=30)
cubic_pts = cubic_spline(path, samples_per_segment=30)
bspline_pts = bspline_curve(path, degree=3, samples=200)

# Compute metrics
lengths = {
    'raw': path_length(path),
    'bezier': path_length(bezier_pts),
    'cubic': path_length(cubic_pts),
    'bspline': path_length(bspline_pts)
}
```

## Output Example

```
======================================================================
Adaptive Spline Selection Demo (Sport Mode = Shortest Path)
======================================================================

Grid: 8 x 4
Obstacles: {(2, 1), (5, 2)}

Lazy Theta* path found with 3 nodes
Path waypoints: [(0, 0), (2, 2), (8, 4)]

Path divided into 2 segments:
  Segment(0, (0, 0) -> (2, 2), L=2.83)
  Segment(1, (2, 2) -> (8, 4), L=6.32)

Path Length Statistics:
----------------------------------------------------------------------
Raw Path            : Length=   9.153  Points=    3
Bezier Spline       : Length=   9.198  Points=   61
Cubic Spline        : Length=   9.219  Points=   61
B-Spline            : Length=   9.010  Points=  201
----------------------------------------------------------------------

Adaptive Selection (SPORT mode = minimize length):
----------------------------------------------------------------------
  Segment 0: Segment(0, (0, 0) -> (2, 2), L=2.83) -> chosen: bspline  (cost=9.010)
  Segment 1: Segment(1, (2, 2) -> (8, 4), L=6.32) -> chosen: bspline  (cost=9.010)

Total adaptive path length (estimated): 9.010
```

## Future Extensions

### New Selection Modes
To add "EFFICIENT" or "NORMAL" modes:
1. Implement energy calculation function
2. Update `AdaptiveSelector.compute_cost()` in `segment.py`
3. Add mode-specific cost metrics

### Per-Segment Spline Evaluation
Currently, the full spline length is used for all segments. Future improvements:
- Evaluate each spline only on its segment
- Implement window-based smoothing
- Compute segment-local curvature metrics

### Performance Optimization
- Memoize Cox-de Boor basis computations
- Vectorize spline evaluation using NumPy
- Implement adaptive sampling based on local curvature

### Testing
- Add unit tests for each library module
- Test with various grid sizes and obstacle configurations
- Benchmark performance on large paths

## File Structure

```
algorithms/
├── lib/                      # Modular libraries
│   ├── __init__.py
│   ├── pathfinding.py        # Lazy Theta* and utilities
│   ├── bezier_lib.py         # Bezier spline
│   ├── cubic_lib.py          # Cubic spline
│   ├── bspline_lib.py        # B-spline
│   ├── segment.py            # Segments and adaptive selection
│   └── visualization.py      # Plotting utilities
├── main.py                   # Main integration script
├── outputs/                  # Generated images
│   ├── raw_path.png
│   └── adaptive_spline_comparison.png
└── README.md                 # This file
```

## Dependencies

- **Python 3.7+**
- **matplotlib** - For plotting/visualization
- Standard library: `math`, `heapq`, `typing`, `os`, `sys`, `enum`

## Notes

- The GTK warning in console output can be safely ignored (matplotlib backend issue)
- All path coordinates are floating-point (X, Y) tuples
- Grid indices are integers; grid is inclusive [0, width] × [0, height]
- Obstacles are defined as integer coordinate tuples

## Author

Tomás Barros
