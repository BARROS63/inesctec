"""
Main integration script: Demonstrates the adaptive spline selection algorithm.

This script:
1. Sets up a 2D grid with obstacles
2. Computes the shortest path using Lazy Theta*
3. Applies three different spline smoothing methods
4. Uses adaptive selection (sport mode) to choose the best spline for each segment
5. Produces a comparison image and prints path length statistics
"""

import sys
import os
from typing import List, Tuple, Dict, Set

# Add lib directory to path
lib_dir = os.path.join(os.path.dirname(__file__), 'lib')
sys.path.insert(0, lib_dir)

from pathfinding import LazyThetaStar, path_length
from bezier_lib import bezier_spline
from cubic_lib import cubic_spline
from bspline_lib import bspline_curve
from segment import (create_segments, PathSegment, AdaptiveSelector, 
                     SelectionMode, compute_curvature_for_segment, segment_length)
from visualization import (plot_raw_path, plot_spline_comparison, 
                          print_path_statistics)


def main():
    """Main demonstration function."""
    
    # --- Setup: Define grid and obstacles ---
    width = 8
    height = 4
    obstacles: Set[Tuple[int, int]] = {(2, 1), (5, 2)}
    
    print("=" * 70)
    print("Adaptive Spline Selection Demo (Sport Mode)")
    print("=" * 70)
    print(f"\nGrid: {width} x {height}")
    print(f"Obstacles: {obstacles}\n")
    
    # --- Pathfinding ---
    lts = LazyThetaStar(width, height, obstacles)
    start = (0, 0)
    goal = (width, height)
    
    path = lts.compute_path(start, goal)
    if not path:
        print("ERROR: No path found!")
        return
    
    print(f"Lazy Theta* path found with {len(path)} nodes")
    print(f"Path waypoints: {path}\n")
    
    # --- Convert path to segments ---
    segments = create_segments(path)
    print(f"Path divided into {len(segments)} segments:")
    for seg in segments:
        print(f"  {seg}")
    print()
    
    # --- Generate spline smoothings ---
    print("Generating splines...")
    bezier_pts = bezier_spline(path, samples_per_segment=30)
    cubic_pts = cubic_spline(path, samples_per_segment=30)
    bspline_pts = bspline_curve(path, degree=3, samples=200, clamped=True)
    
    # --- Compute metrics ---
    raw_length = path_length(path)
    bezier_length = path_length(bezier_pts)
    cubic_length = path_length(cubic_pts)
    bspline_length = path_length(bspline_pts)
    
    print("\nPath Length Statistics:")
    print("-" * 70)
    print_path_statistics("Raw Path", raw_length, len(path))
    print_path_statistics("Bezier Spline", bezier_length, len(bezier_pts))
    print_path_statistics("Cubic Spline", cubic_length, len(cubic_pts))
    print_path_statistics("B-Spline", bspline_length, len(bspline_pts))
    print("-" * 70)
    
    # --- Adaptive Selection (Sport Mode) ---
    print("\nAdaptive Selection (SPORT mode = minimize length):")
    print("-" * 70)
    
    selector = AdaptiveSelector(mode=SelectionMode.SPORT)
    gamma = 0.5  # Weight for curvature term
    
    # For each segment, compute candidate costs and select best
    segment_choices: Dict[int, str] = {}  # segment_id -> chosen spline name
    total_selected_length = 0.0
    
    for i, seg in enumerate(segments):
        # Get segment points from each spline
        # (Note: in a full implementation, we'd evaluate each spline on just this segment)
        # For this demo, we'll compute a curvature-weighted cost across the full spline
        
        prev_seg = segments[i - 1] if i > 0 else None
        next_seg = segments[i + 1] if i < len(segments) - 1 else None
        
        curvature = compute_curvature_for_segment(prev_seg, next_seg)
        
        # Create candidate evaluations (in practice, each spline would be evaluated per-segment)
        # For demo, use the overall metrics
        candidates = {
            'bezier': (bezier_length, curvature),
            'cubic': (cubic_length, curvature),
            'bspline': (bspline_length, curvature)
        }
        
        best_name, best_cost = selector.select_best(candidates, gamma=gamma)
        segment_choices[i] = best_name
        
        # Get the length contribution (simplified for demo)
        if best_name == 'bezier':
            seg_contribution = bezier_length / len(segments)
        elif best_name == 'cubic':
            seg_contribution = cubic_length / len(segments)
        else:  # bspline
            seg_contribution = bspline_length / len(segments)
        
        total_selected_length += seg_contribution
        print(f"  Segment {i}: {seg} -> chosen: {best_name:8s} (cost={best_cost:.3f})")
    
    print(f"\nTotal adaptive path length (estimated): {total_selected_length:.3f}")
    print("-" * 70)
    
    # --- Generate comparison image ---
    output_dir = os.path.join(os.path.dirname(__file__), 'outputs')
    os.makedirs(output_dir, exist_ok=True)
    
    spline_dict = {
        'bezier': bezier_pts,
        'cubic': cubic_pts,
        'bspline': bspline_pts
    }
    
    length_dict = {
        'raw': raw_length,
        'bezier': bezier_length,
        'cubic': cubic_length,
        'bspline': bspline_length
    }
    
    comparison_file = os.path.join(output_dir, 'adaptive_spline_comparison.png')
    plot_spline_comparison(width, height, obstacles, path, spline_dict, 
                          length_dict, comparison_file)
    
    print("\n" + "=" * 70)

if __name__ == '__main__':
    main()
