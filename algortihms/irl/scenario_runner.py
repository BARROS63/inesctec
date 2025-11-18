"""
Real-world scenario runner with GPS coordinates, energy models, and path mode selection.

This script allows users to:
1. Select or define a scenario with real GPS coordinates
2. Choose a path optimization mode (sport, normal, efficient)
3. Run the pathfinding and spline algorithms
4. Compare energy consumption across modes
5. Generate waypoints and mission file

Usage:
    python3 scenario_runner.py --scenario campus --mode sport
    python3 scenario_runner.py --interactive
"""

import sys
import os

# Add lib directory to path FIRST
lib_dir = os.path.join(os.path.dirname(__file__), 'lib')
if lib_dir not in sys.path:
    sys.path.insert(0, lib_dir)

import argparse
import json
import math
from typing import List, Tuple, Dict, Set
from enum import Enum

from pathfinding import LazyThetaStar, path_length, euclidean
from bezier_lib import bezier_spline
from cubic_lib import cubic_spline
from bspline_lib import bspline_curve
from segment import AdaptiveSelector, SelectionMode, compute_curvature_for_segment, create_segments
from visualization import plot_spline_comparison, print_path_statistics
from realworld import (
    GPSCoordinate, ScenarioConfiguration, EnergyCalculator, 
    PathMode, DRONE_PROFILES, AVAILABLE_SCENARIOS
)


def compute_path_metrics(path: List[Tuple[float, float]]) -> Dict[str, float]:
    """Compute path metrics (curvature, smoothness, etc.)."""
    if len(path) < 2:
        return {'total_curvature': 0.0, 'avg_segment_length': 0.0}
    
    total_curvature = 0.0
    for i in range(1, len(path) - 1):
        # Vector from point i-1 to i
        v1 = (path[i][0] - path[i-1][0], path[i][1] - path[i-1][1])
        # Vector from point i to i+1
        v2 = (path[i+1][0] - path[i][0], path[i+1][1] - path[i][1])
        
        # Compute angle between vectors
        len1 = euclidean(path[i-1], path[i])
        len2 = euclidean(path[i], path[i+1])
        
        if len1 > 0 and len2 > 0:
            dot = v1[0] * v2[0] + v1[1] * v2[1]
            det = v1[0] * v2[1] - v1[1] * v2[0]
            angle = abs(math.atan2(det, dot))
            total_curvature += angle
    
    avg_seg_length = path_length(path) / (len(path) - 1) if len(path) > 1 else 0.0
    
    return {
        'total_curvature': total_curvature,
        'avg_segment_length': avg_seg_length,
        'num_waypoints': len(path)
    }


import math


def run_scenario(scenario: ScenarioConfiguration, 
                 mode: PathMode,
                 mode_weights: Dict[PathMode, float]) -> Dict:
    """
    Run a complete scenario with selected mode.
    
    Args:
        scenario: Configured scenario with GPS coordinates
        mode: Selected path mode (sport, normal, efficient)
        mode_weights: Weights for cost function per mode
    
    Returns:
        Results dictionary with paths, energies, and comparisons
    """
    
    print("=" * 80)
    print(f"Real-World Scenario: {scenario.name}")
    print(f"Drone: {scenario.drone}")
    print(f"Mode: {mode.value.upper()}")
    print("=" * 80)
    
    # Convert GPS to grid
    try:
        start_grid, goal_grid, obstacles_grid, width, height = scenario.get_grid_coordinates()
    except ValueError as e:
        print(f"ERROR: {e}")
        return {}
    
    print(f"\nCoordinate Conversion:")
    print(f"  Start (GPS): {scenario.start_gps}")
    print(f"  Start (Grid): {start_grid}")
    print(f"  Goal (GPS): {scenario.goal_gps}")
    print(f"  Goal (Grid): {goal_grid}")
    print(f"  Grid Size: {width} x {height}")
    print(f"  Obstacles (grid): {obstacles_grid}\n")
    
    # Compute path with Lazy Theta*
    lts = LazyThetaStar(width, height, set(obstacles_grid))
    path = lts.compute_path(start_grid, goal_grid)
    
    if not path:
        print("ERROR: No path found!")
        return {}
    
    print(f"Path computed with {len(path)} waypoints")
    print(f"Raw path length: {path_length(path):.2f} grid units\n")
    
    # Apply spline smoothing
    print("Generating smoothed paths...")
    bezier_pts = bezier_spline(path, samples_per_segment=30)
    cubic_pts = cubic_spline(path, samples_per_segment=30)
    bspline_pts = bspline_curve(path, degree=3, samples=200, clamped=True)
    
    # Compute metrics for each spline
    raw_length = path_length(path)
    bezier_length = path_length(bezier_pts)
    cubic_length = path_length(cubic_pts)
    bspline_length = path_length(bspline_pts)
    
    # Compute curvature metrics
    raw_metrics = compute_path_metrics(path)
    
    print("\nPath Metrics:")
    print("-" * 80)
    print(f"{'Spline Type':<15} {'Length':<12} {'Waypoints':<12} {'Curvature':<15}")
    print("-" * 80)
    print(f"{'Raw Path':<15} {raw_length:<12.3f} {len(path):<12} {raw_metrics['total_curvature']:<15.4f}")
    print(f"{'Bezier':<15} {bezier_length:<12.3f} {len(bezier_pts):<12} {'N/A':<15}")
    print(f"{'Cubic':<15} {cubic_length:<12.3f} {len(cubic_pts):<12} {'N/A':<15}")
    print(f"{'B-Spline':<15} {bspline_length:<12.3f} {len(bspline_pts):<12} {'N/A':<15}")
    print("-" * 80)
    
    # Select best spline based on mode
    print("\nSpline Selection:")
    print("-" * 80)
    
    selector = AdaptiveSelector(mode=SelectionMode.SPORT)
    gamma = mode_weights.get(mode, 0.5)
    
    candidates = {
        'bezier': (bezier_length, raw_metrics['total_curvature']),
        'cubic': (cubic_length, raw_metrics['total_curvature']),
        'bspline': (bspline_length, raw_metrics['total_curvature'])
    }
    
    best_name, best_cost = selector.select_best(candidates, gamma=gamma)
    print(f"Best spline for {mode.value}: {best_name} (cost={best_cost:.3f})")
    
    # Select the chosen spline points
    spline_choices = {
        'bezier': bezier_pts,
        'cubic': cubic_pts,
        'bspline': bspline_pts
    }
    chosen_path = spline_choices[best_name]
    
    print("-" * 80)
    
    # Convert path back to GPS coordinates
    print("\nPath Generation:")
    print("-" * 80)
    
    # Compute altitude variation
    alt_start = scenario.start_gps.altitude
    alt_goal = scenario.goal_gps.altitude
    alt_variation = alt_goal - alt_start
    
    # Generate GPS waypoints
    gps_waypoints = []
    for i, (grid_x, grid_y) in enumerate(chosen_path):
        # Interpolate altitude
        progress = i / (len(chosen_path) - 1) if len(chosen_path) > 1 else 0
        altitude = alt_start + (alt_variation * progress)
        
        gps_point = scenario.converter.grid_to_gps(int(grid_x), int(grid_y), altitude)
        gps_waypoints.append(gps_point)
    
    print(f"Generated {len(gps_waypoints)} GPS waypoints")
    print(f"Altitude range: {alt_start:.1f}m to {alt_goal:.1f}m\n")
    
    # Compute energy for different modes
    print("Energy Consumption Analysis:")
    print("-" * 80)
    
    calculator = EnergyCalculator(scenario.drone)
    path_length_m = path_length(chosen_path) * scenario.grid_size_m  # Convert to meters
    
    energy_sport = calculator.compute_energy_sport(path_length_m, abs(alt_variation))
    energy_normal = calculator.compute_energy_normal(
        path_length_m, abs(alt_variation),
        path_curvature=1.0 + raw_metrics['total_curvature'] / 10
    )
    energy_efficient = calculator.compute_energy_efficient(
        path_length_m, abs(alt_variation),
        path_curvature=1.0 + raw_metrics['total_curvature'] / 10,
        path_smoothness=1.0 / (1.0 + raw_metrics['total_curvature'] / 10)
    )
    
    print(f"{'Mode':<15} {'Energy (Wh)':<15} {'Time (min)':<15} {'Battery %':<15}")
    print("-" * 80)
    print(f"{'SPORT':<15} {energy_sport['energy_Wh']:<15.2f} {energy_sport['time_minutes']:<15.2f} {energy_sport['battery_percentage']:<15.1f}%")
    print(f"{'NORMAL':<15} {energy_normal['energy_Wh']:<15.2f} {energy_normal['time_minutes']:<15.2f} {energy_normal['battery_percentage']:<15.1f}%")
    print(f"{'EFFICIENT':<15} {energy_efficient['energy_Wh']:<15.2f} {energy_efficient['time_minutes']:<15.2f} {energy_efficient['battery_percentage']:<15.1f}%")
    print("-" * 80)
    
    # Hover time reference
    hover_time = calculator.hovering_time_minutes()
    if hover_time > 0:
        print(f"Reference: Hovering time = {hover_time:.1f} minutes\n")
    
    return {
        'scenario': scenario,
        'mode': mode,
        'raw_path': path,
        'chosen_path': chosen_path,
        'gps_waypoints': gps_waypoints,
        'path_length_grid': path_length(chosen_path),
        'path_length_m': path_length_m,
        'energy': {
            'sport': energy_sport,
            'normal': energy_normal,
            'efficient': energy_efficient
        },
        'spline_choice': best_name,
        'spline_points': {
            'bezier': bezier_pts,
            'cubic': cubic_pts,
            'bspline': bspline_pts
        }
    }


def save_mission_file(results: Dict, filename: str):
    """Save waypoints as a mission file (JSON format for easy parsing)."""
    
    mission = {
        'metadata': {
            'scenario': results['scenario'].name,
            'drone': str(results['scenario'].drone),
            'mode': results['mode'].value,
            'spline_used': results['spline_choice']
        },
        'path_info': {
            'total_waypoints': len(results['gps_waypoints']),
            'path_length_m': results['path_length_m'],
            'energy_analysis': {
                mode: {
                    'energy_Wh': results['energy'][mode]['energy_Wh'],
                    'time_minutes': results['energy'][mode]['time_minutes'],
                    'battery_percentage': results['energy'][mode]['battery_percentage']
                }
                for mode in ['sport', 'normal', 'efficient']
            }
        },
        'waypoints': [
            {
                'id': i,
                'latitude': wp.latitude,
                'longitude': wp.longitude,
                'altitude': wp.altitude
            }
            for i, wp in enumerate(results['gps_waypoints'])
        ]
    }
    
    with open(filename, 'w') as f:
        json.dump(mission, f, indent=2)
    
    print(f"Mission file saved: {filename}")


def save_mission_planner_file(results: Dict, filename: str):
    """
    Save mission file in Mission Planner format (QGC/ArduPilot compatible).
    
    Format: QGroundControl WPL 120
    Each waypoint: seq, current, frame, command, param1-4, x(lat), y(lon), z(alt), autocontinue
    """
    waypoints = results['gps_waypoints']
    
    lines = ["QGC WPL 110"]  # Header for QGroundControl Mission Planner
    
    for i, wp in enumerate(waypoints):
        seq = i
        current = 1 if i == 0 else 0  # Mark first waypoint as current
        frame = 3  # 3 = global relative altitude (most common)
        command = 16  # 16 = MAV_CMD_NAV_WAYPOINT
        param1 = 0  # Hold time (0 = no hold)
        param2 = 2.0  # Acceptance radius in meters
        param3 = 0  # Pass through waypoint
        param4 = 0  # Desired yaw angle (0 = don't change)
        lat = wp.latitude
        lon = wp.longitude
        alt = wp.altitude
        autocontinue = 1  # Auto-continue to next waypoint
        
        # QGC format: tab-separated values
        line = f"{seq}\t{current}\t{frame}\t{command}\t{param1}\t{param2}\t{param3}\t{param4}\t{lat}\t{lon}\t{alt}\t{autocontinue}"
        lines.append(line)
    
    with open(filename, 'w') as f:
        f.write('\n'.join(lines))
    
    print(f"Mission Planner file (.txt) saved: {filename}")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Real-world UAV path planning with adaptive spline selection'
    )
    parser.add_argument('--scenario', type=str, default='campus',
                       choices=list(AVAILABLE_SCENARIOS.keys()),
                       help='Predefined scenario to run')
    parser.add_argument('--mode', type=str, default='sport',
                       choices=['sport', 'normal', 'efficient'],
                       help='Path optimization mode')
    parser.add_argument('--interactive', action='store_true',
                       help='Run in interactive mode (custom scenario)')
    parser.add_argument('--drone', type=str, default='quadcopter_medium',
                       choices=list(DRONE_PROFILES.keys()),
                       help='Drone profile to use')
    parser.add_argument('--save-mission', type=str,
                       help='Save waypoints to JSON mission file')
    parser.add_argument('--save-mp', type=str, metavar='FILE.txt',
                       help='Save waypoints to Mission Planner format (.txt)')
    
    args = parser.parse_args()
    
    # Mode weights for cost function
    mode_weights = {
        PathMode.SPORT: 0.1,      # Minimize curvature penalty
        PathMode.NORMAL: 0.5,     # Balanced
        PathMode.EFFICIENT: 1.0   # Maximize smoothness preference
    }
    
    # Load or create scenario
    if args.interactive:
        print("Interactive mode not yet implemented. Using default scenario.")
        scenario_fn = AVAILABLE_SCENARIOS['campus']
        scenario = scenario_fn()
    else:
        scenario_fn = AVAILABLE_SCENARIOS[args.scenario]
        scenario = scenario_fn()
        
        # Override drone if specified
        if args.drone != 'quadcopter_medium':
            scenario.drone = DRONE_PROFILES[args.drone]
    
    # Run scenario
    mode = PathMode[args.mode.upper()]
    results = run_scenario(scenario, mode, mode_weights)
    
    # Save mission file if requested
    if args.save_mission and results:
        save_mission_file(results, args.save_mission)
    
    # Save Mission Planner format if requested
    if args.save_mp and results:
        save_mission_planner_file(results, args.save_mp)
    
    print("\n" + "=" * 80)
    print("Scenario completed successfully!")
    print("=" * 80)


if __name__ == '__main__':
    main()
