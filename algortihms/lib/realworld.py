"""
Real-world scenario module: Handles GPS coordinates, obstacle definitions, and energy models.

This module provides:
1. Real-world coordinate input (latitude, longitude, altitude)
2. Grid conversion and obstacle definition
3. Drone-specific power consumption models for different path types
4. Path mode selection (sport, normal, efficient)
"""

import math
from typing import Tuple, List, Dict, Optional
from enum import Enum
from dataclasses import dataclass


class PathMode(Enum):
    """Path optimization modes with drone-specific energy models."""
    SPORT = "sport"              # Fastest/shortest path
    NORMAL = "normal"            # Balanced approach
    EFFICIENT = "efficient"      # Most energy-efficient


@dataclass
class GPSCoordinate:
    """Represents a GPS coordinate with altitude."""
    latitude: float      # degrees (-90 to 90)
    longitude: float     # degrees (-180 to 180)
    altitude: float      # meters above sea level
    
    def __repr__(self) -> str:
        return f"GPS({self.latitude:.6f}, {self.longitude:.6f}, {self.altitude:.1f}m)"


@dataclass
class DroneProfile:
    """Drone specifications for energy calculations."""
    mass_kg: float                    # Total mass in kg
    max_speed_ms: float              # Maximum speed m/s
    cruise_speed_ms: float           # Optimal cruise speed m/s
    battery_capacity_Wh: float       # Battery energy Wh
    motor_efficiency: float          # Motor efficiency (0-1)
    hover_power_W: float             # Power needed to hover
    
    def __repr__(self) -> str:
        return f"Drone(m={self.mass_kg}kg, v_cruise={self.cruise_speed_ms}m/s, " \
               f"battery={self.battery_capacity_Wh}Wh)"


# Common drone profiles
DRONE_PROFILES = {
    'quadcopter_light': DroneProfile(
        mass_kg=0.5,
        max_speed_ms=15.0,
        cruise_speed_ms=8.0,
        battery_capacity_Wh=1000.0,
        motor_efficiency=0.85,
        hover_power_W=50.0
    ),
    'quadcopter_medium': DroneProfile(
        mass_kg=2.0,
        max_speed_ms=20.0,
        cruise_speed_ms=12.0,
        battery_capacity_Wh=5000.0,
        motor_efficiency=0.82,
        hover_power_W=180.0
    ),
    'quadcopter_heavy': DroneProfile(
        mass_kg=5.0,
        max_speed_ms=18.0,
        cruise_speed_ms=10.0,
        battery_capacity_Wh=15000.0,
        motor_efficiency=0.80,
        hover_power_W=500.0
    ),
    'fixed_wing': DroneProfile(
        mass_kg=1.5,
        max_speed_ms=25.0,
        cruise_speed_ms=18.0,
        battery_capacity_Wh=3000.0,
        motor_efficiency=0.88,
        hover_power_W=0.0  # Fixed wings don't hover
    )
}


class CoordinateConverter:
    """Converts GPS coordinates to grid coordinates and vice versa."""
    
    def __init__(self, origin_gps: GPSCoordinate, 
                 grid_size_m: float = 1.0,
                 earth_radius_m: float = 6371000.0):
        """
        Initialize coordinate converter.
        
        Args:
            origin_gps: Reference GPS point (0,0) in grid
            grid_size_m: Meters per grid cell
            earth_radius_m: Earth radius for distance calculations
        """
        self.origin_gps = origin_gps
        self.grid_size_m = grid_size_m
        self.earth_radius_m = earth_radius_m
    
    def gps_to_grid(self, gps: GPSCoordinate) -> Tuple[int, int, float]:
        """
        Convert GPS coordinate to grid coordinate.
        
        Returns:
            Tuple of (grid_x, grid_y, altitude_diff) where altitude_diff is in meters
        """
        # Haversine formula for distance
        lat1_rad = math.radians(self.origin_gps.latitude)
        lat2_rad = math.radians(gps.latitude)
        dlon_rad = math.radians(gps.longitude - self.origin_gps.longitude)
        dlat_rad = math.radians(gps.latitude - self.origin_gps.latitude)
        
        # Distance in north direction
        a = math.sin(dlat_rad / 2) ** 2
        c = 2 * math.asin(math.sqrt(a))
        north_dist_m = self.earth_radius_m * c
        
        # Distance in east direction
        a = math.sin(dlon_rad / 2) ** 2
        c = 2 * math.asin(math.sqrt(a * math.cos(lat1_rad) * math.cos(lat2_rad)))
        east_dist_m = self.earth_radius_m * c
        
        # Convert to grid coordinates
        grid_x = int(round(east_dist_m / self.grid_size_m))
        grid_y = int(round(north_dist_m / self.grid_size_m))
        alt_diff = gps.altitude - self.origin_gps.altitude
        
        return (grid_x, grid_y, alt_diff)
    
    def grid_to_gps(self, grid_x: int, grid_y: int, 
                    altitude_m: float) -> GPSCoordinate:
        """Convert grid coordinate back to GPS."""
        # Distance in meters
        north_dist_m = grid_y * self.grid_size_m
        east_dist_m = grid_x * self.grid_size_m
        
        # Convert to degrees (approximate for small distances)
        lat_offset = north_dist_m / self.earth_radius_m * (180 / math.pi)
        lon_offset = east_dist_m / (self.earth_radius_m * 
                                    math.cos(math.radians(self.origin_gps.latitude))) * (180 / math.pi)
        
        return GPSCoordinate(
            latitude=self.origin_gps.latitude + lat_offset,
            longitude=self.origin_gps.longitude + lon_offset,
            altitude=altitude_m
        )


class EnergyCalculator:
    """Calculates drone energy consumption for different path types."""
    
    def __init__(self, drone: DroneProfile):
        self.drone = drone
        self.g = 9.81  # gravity m/s^2
    
    def hovering_time_minutes(self) -> float:
        """Estimate hovering time in minutes."""
        if self.drone.hover_power_W == 0:
            return 0  # Fixed wings can't hover
        hovering_time_s = (self.drone.battery_capacity_Wh * 3600) / self.drone.hover_power_W
        return hovering_time_s / 60
    
    def compute_energy_sport(self, path_length_m: float, 
                             path_height_variation_m: float = 0) -> Dict[str, float]:
        """
        SPORT mode: Minimize TIME, accept higher energy consumption.
        
        IMPORTANT NOTE ON ENERGY:
        Power P = F·v/η scales linearly with velocity.
        Therefore: SPORT mode DOES NOT minimize energy!
        
        Sport uses 85% max speed:
        - Pro: Minimum flight time
        - Con: HIGH energy due to high speed (P ∝ v)
        - Use case: Time-critical missions, racing, fast delivery when energy budget allows
        
        Energy comparison: Sport typically HIGHER than efficient despite shorter path,
        because the power penalty of high speed outweighs the distance savings.
        """
        # Speed: 85% of max speed (aggressive but safe)
        speed_ms = self.drone.max_speed_ms * 0.85
        
        # Flight time: horizontal component
        horizontal_time_s = path_length_m / speed_ms if speed_ms > 0 else 0
        
        # Vertical component: fast climb
        climb_rate_ms = 3.0  # m/s for sport mode
        vertical_time_s = abs(path_height_variation_m) / climb_rate_ms if path_height_variation_m != 0 else 0
        
        total_time_s = horizontal_time_s + vertical_time_s
        
        # Energy calculation: P = F·v / η (power scales with speed!)
        thrust_N = self.drone.mass_kg * self.g
        
        # Horizontal propulsion power - HIGH at high speed
        propulsion_power_W = (thrust_N * speed_ms) / self.drone.motor_efficiency
        
        # Vertical propulsion (if needed)
        if path_height_variation_m != 0:
            vertical_power_W = (thrust_N * climb_rate_ms) / self.drone.motor_efficiency
        else:
            vertical_power_W = 0
        
        # Maneuver overhead: 15% extra for aggressive turns at high speed
        maneuver_overhead = 1.15
        
        total_power_W = (propulsion_power_W + vertical_power_W) * maneuver_overhead
        energy_Wh = (total_power_W * total_time_s) / 3600
        
        return {
            'energy_Wh': energy_Wh,
            'time_minutes': total_time_s / 60,
            'speed_ms': speed_ms,
            'path_length_m': path_length_m,
            'battery_percentage': (energy_Wh / self.drone.battery_capacity_Wh) * 100,
            'optimization': 'TIME (not energy)'
        }
    
    def compute_energy_normal(self, path_length_m: float, 
                              path_height_variation_m: float = 0,
                              path_curvature: float = 1.0) -> Dict[str, float]:
        """
        NORMAL mode: Balanced between speed and efficiency.
        
        Uses cruise speed and considers path smoothness.
        Curvature factor: 1.0 = no extra turns, >1.0 = increased turns
        """
        # Use cruise speed (optimal efficiency-speed tradeoff)
        speed_ms = self.drone.cruise_speed_ms
        
        # Apply curvature penalty: more turns = longer effective distance
        effective_distance_m = path_length_m * path_curvature
        
        # Flight time
        horizontal_time_s = effective_distance_m / speed_ms if speed_ms > 0 else 0
        
        # Vertical component
        climb_rate_ms = 2.0  # m/s for normal mode
        vertical_time_s = abs(path_height_variation_m) / climb_rate_ms if path_height_variation_m != 0 else 0
        
        total_time_s = horizontal_time_s + vertical_time_s  # Sequential climbing+flying
        
        # Energy calculation
        thrust_N = self.drone.mass_kg * self.g
        
        # Horizontal propulsion
        propulsion_power_W = (thrust_N * speed_ms) / self.drone.motor_efficiency
        
        # Vertical propulsion (if needed)
        if path_height_variation_m != 0:
            climb_rate_ms = 2.0
            vertical_power_W = (thrust_N * climb_rate_ms) / self.drone.motor_efficiency
        else:
            vertical_power_W = 0
        
        # Efficiency bonus for smoother paths
        curvature_penalty = 1.0 + (path_curvature - 1.0) * 0.05
        
        total_power_W = (propulsion_power_W + vertical_power_W) * curvature_penalty
        energy_Wh = (total_power_W * total_time_s) / 3600
        
        return {
            'energy_Wh': energy_Wh,
            'time_minutes': total_time_s / 60,
            'speed_ms': speed_ms,
            'path_length_m': effective_distance_m,
            'battery_percentage': (energy_Wh / self.drone.battery_capacity_Wh) * 100,
            'curvature_factor': path_curvature
        }
    
    def compute_energy_efficient(self, path_length_m: float, 
                                 path_height_variation_m: float = 0,
                                 path_curvature: float = 1.0,
                                 path_smoothness: float = 1.0) -> Dict[str, float]:
        """
        EFFICIENT mode: Minimize energy consumption, accept longer time.
        
        Uses slower speeds, prefers smoother paths to reduce motor strain.
        
        Args:
            path_length_m: Total path distance in meters
            path_height_variation_m: Total altitude change in meters
            path_curvature: Ratio of actual path to straight line (1.0 = straight)
            path_smoothness: Inverse of curvature (1.0 = straight, <1.0 = curved)
        """
        # Use 60% of cruise speed for maximum efficiency
        speed_ms = self.drone.cruise_speed_ms * 0.60
        
        # Prefer smoother paths: reduce distance by smoothness factor
        effective_distance_m = path_length_m * path_smoothness
        
        # Flight time
        horizontal_time_s = effective_distance_m / speed_ms if speed_ms > 0 else 0
        
        # Vertical component: very gentle climb for efficiency
        climb_rate_ms = 1.0  # m/s for efficient mode
        vertical_time_s = abs(path_height_variation_m) / climb_rate_ms if path_height_variation_m != 0 else 0
        
        total_time_s = horizontal_time_s + vertical_time_s
        
        # Energy calculation: optimized for efficiency
        thrust_N = self.drone.mass_kg * self.g
        
        # Lower speed = lower power requirement (P = F*v)
        propulsion_power_W = (thrust_N * speed_ms) / self.drone.motor_efficiency
        
        # Vertical propulsion
        if path_height_variation_m != 0:
            vertical_power_W = (thrust_N * climb_rate_ms) / self.drone.motor_efficiency
        else:
            vertical_power_W = 0
        
        # Smooth paths reduce motor strain: 5% bonus for each 0.1 smoothness improvement
        smoothness_bonus = max(0.8, 1.0 - (1.0 - path_smoothness) * 0.5)
        
        total_power_W = (propulsion_power_W + vertical_power_W) * smoothness_bonus
        energy_Wh = (total_power_W * total_time_s) / 3600
        
        return {
            'energy_Wh': energy_Wh,
            'time_minutes': total_time_s / 60,
            'speed_ms': speed_ms,
            'path_length_m': effective_distance_m,
            'battery_percentage': (energy_Wh / self.drone.battery_capacity_Wh) * 100,
            'smoothness_factor': path_smoothness
        }


class ScenarioConfiguration:
    """Manages a real-world flight scenario."""
    
    def __init__(self, name: str, drone_profile: Optional[DroneProfile] = None,
                 grid_size_m: float = 10.0):
        self.name = name
        self.drone = drone_profile or DRONE_PROFILES['quadcopter_medium']
        self.grid_size_m = grid_size_m
        self.converter: Optional[CoordinateConverter] = None
        self.start_gps: Optional[GPSCoordinate] = None
        self.goal_gps: Optional[GPSCoordinate] = None
        self.obstacles_gps: List[GPSCoordinate] = []
    
    def set_origin(self, gps: GPSCoordinate):
        """Set the origin for coordinate conversion."""
        self.converter = CoordinateConverter(gps, self.grid_size_m)
    
    def add_start_point(self, gps: GPSCoordinate):
        """Set the start point in GPS coordinates."""
        self.start_gps = gps
    
    def add_goal_point(self, gps: GPSCoordinate):
        """Set the goal point in GPS coordinates."""
        self.goal_gps = gps
    
    def add_obstacle(self, gps: GPSCoordinate, radius_m: float = 50.0):
        """Add an obstacle (circular region) in GPS coordinates."""
        self.obstacles_gps.append(gps)
    
    def get_grid_coordinates(self) -> Tuple[Tuple[int, int], Tuple[int, int], 
                                           List[Tuple[int, int]], int, int]:
        """
        Convert GPS scenario to grid coordinates.
        
        Returns:
            Tuple of (start_grid, goal_grid, obstacles_grid, width, height)
        """
        if not self.converter or not self.start_gps or not self.goal_gps:
            raise ValueError("Origin, start point, and goal point must be set")
        
        start_grid = self.converter.gps_to_grid(self.start_gps)[:2]
        goal_grid = self.converter.gps_to_grid(self.goal_gps)[:2]
        
        obstacles_grid = [self.converter.gps_to_grid(obs)[:2] for obs in self.obstacles_gps]
        
        # Compute grid size
        all_x = [start_grid[0], goal_grid[0]] + [o[0] for o in obstacles_grid]
        all_y = [start_grid[1], goal_grid[1]] + [o[1] for o in obstacles_grid]
        
        width = max(all_x) + 5
        height = max(all_y) + 5
        
        return (start_grid, goal_grid, obstacles_grid, width, height)
    
    def __repr__(self) -> str:
        return f"Scenario({self.name}, {self.drone})"


# Add at end of realworld.py

def create_campus_survey_scenario():
    """Campus survey scenario."""
    scenario = ScenarioConfiguration(name="Campus Survey", drone_profile=DRONE_PROFILES['quadcopter_medium'], grid_size_m=10.0)
    origin = GPSCoordinate(latitude=41.159, longitude=-8.629, altitude=90.0)
    scenario.set_origin(origin)
    scenario.add_start_point(GPSCoordinate(latitude=41.159, longitude=-8.629, altitude=95.0))
    scenario.add_goal_point(GPSCoordinate(latitude=41.160, longitude=-8.631, altitude=115.0))
    scenario.add_obstacle(GPSCoordinate(latitude=41.1592, longitude=-8.6295, altitude=85.0), radius_m=30.0)
    scenario.add_obstacle(GPSCoordinate(latitude=41.1595, longitude=-8.6305, altitude=88.0), radius_m=25.0)
    return scenario

def create_agricultural_survey_scenario():
    """Agricultural survey scenario."""
    scenario = ScenarioConfiguration(name="Agricultural Survey", drone_profile=DRONE_PROFILES['quadcopter_heavy'], grid_size_m=20.0)
    origin = GPSCoordinate(latitude=41.15, longitude=-8.70, altitude=50.0)
    scenario.set_origin(origin)
    scenario.add_start_point(GPSCoordinate(latitude=41.15, longitude=-8.70, altitude=55.0))
    scenario.add_goal_point(GPSCoordinate(latitude=41.14, longitude=-8.68, altitude=55.0))
    for i in range(4):
        obs = GPSCoordinate(latitude=41.145 + i * 0.005, longitude=-8.695 + i * 0.003, altitude=45.0)
        scenario.add_obstacle(obs, radius_m=50.0)
    return scenario

def create_urban_delivery_scenario():
    """Urban delivery scenario."""
    scenario = ScenarioConfiguration(name="Urban Delivery", drone_profile=DRONE_PROFILES['quadcopter_light'], grid_size_m=5.0)
    origin = GPSCoordinate(latitude=41.16, longitude=-8.62, altitude=20.0)
    scenario.set_origin(origin)
    scenario.add_start_point(GPSCoordinate(latitude=41.16, longitude=-8.62, altitude=35.0))
    scenario.add_goal_point(GPSCoordinate(latitude=41.165, longitude=-8.625, altitude=25.0))
    buildings = [
        GPSCoordinate(latitude=41.161, longitude=-8.618, altitude=30.0),
        GPSCoordinate(latitude=41.162, longitude=-8.623, altitude=28.0),
        GPSCoordinate(latitude=41.164, longitude=-8.627, altitude=35.0),
    ]
    for building in buildings:
        scenario.add_obstacle(building, radius_m=20.0)
    return scenario

def create_coastal_inspection_scenario():
    """Coastal inspection scenario."""
    scenario = ScenarioConfiguration(name="Coastal Inspection", drone_profile=DRONE_PROFILES['fixed_wing'], grid_size_m=25.0)
    origin = GPSCoordinate(latitude=41.13, longitude=-8.67, altitude=5.0)
    scenario.set_origin(origin)
    scenario.add_start_point(GPSCoordinate(latitude=41.13, longitude=-8.67, altitude=10.0))
    scenario.add_goal_point(GPSCoordinate(latitude=41.12, longitude=-8.665, altitude=15.0))
    obstacles = [
        GPSCoordinate(latitude=41.1270, longitude=-8.6680, altitude=8.0),
        GPSCoordinate(latitude=41.1250, longitude=-8.6670, altitude=10.0),
    ]
    for obs in obstacles:
        scenario.add_obstacle(obs, radius_m=40.0)
    return scenario

AVAILABLE_SCENARIOS = {
    'campus': create_campus_survey_scenario,
    'agriculture': create_agricultural_survey_scenario,
    'delivery': create_urban_delivery_scenario,
    'coastal': create_coastal_inspection_scenario,
}
