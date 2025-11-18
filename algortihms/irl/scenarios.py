"""
Real-world scenario definitions and examples.

This file contains example scenarios for different locations and use cases.
Modify these or create new ones for your own scenarios.
"""

from lib.realworld import (
    GPSCoordinate, ScenarioConfiguration, DRONE_PROFILES, PathMode
)


def create_campus_survey_scenario() -> ScenarioConfiguration:
    """
    Scenario: Campus building survey at INESC TEC.
    
    Flying a quadcopter to survey buildings on campus with obstacles
    representing trees and structures.
    
    Location: Porto, Portugal (approximate)
    Start: Campus entrance (41.159, -8.629, 90m)
    Goal: Main building rooftop (41.160, -8.631, 110m)
    Obstacles: Trees and other structures
    """
    scenario = ScenarioConfiguration(
        name="Campus Survey",
        drone_profile=DRONE_PROFILES['quadcopter_medium'],
        grid_size_m=10.0  # 10m per grid cell
    )
    
    # Set origin at campus entrance
    origin = GPSCoordinate(latitude=41.159, longitude=-8.629, altitude=90.0)
    scenario.set_origin(origin)
    
    # Start point
    start = GPSCoordinate(latitude=41.159, longitude=-8.629, altitude=95.0)
    scenario.add_start_point(start)
    
    # Goal: main building rooftop
    goal = GPSCoordinate(latitude=41.160, longitude=-8.631, altitude=115.0)
    scenario.add_goal_point(goal)
    
    # Obstacles: trees and structures (circular regions approximated as points)
    obstacle1 = GPSCoordinate(latitude=41.1592, longitude=-8.6295, altitude=85.0)
    scenario.add_obstacle(obstacle1, radius_m=30.0)
    
    obstacle2 = GPSCoordinate(latitude=41.1595, longitude=-8.6305, altitude=88.0)
    scenario.add_obstacle(obstacle2, radius_m=25.0)
    
    return scenario

def create_campus_survey_scenario() -> ScenarioConfiguration:
    """
    Scenario: Campus building survey at INESC TEC.
    
    Flying a quadcopter to survey buildings on campus with obstacles
    representing trees and structures.
    
    Location: Porto, Portugal (approximate)
    Start: Campus entrance (41.159, -8.629, 90m)
    Goal: Main building rooftop (41.160, -8.631, 110m)
    Obstacles: Trees and other structures
    """
    scenario = ScenarioConfiguration(
        name="Campus Survey",
        drone_profile=DRONE_PROFILES['quadcopter_medium'],
        grid_size_m=10.0  # 10m per grid cell
    )
    
    # Set origin at campus entrance
    origin = GPSCoordinate(latitude=41.159, longitude=-8.629, altitude=10.0)
    scenario.set_origin(origin)
    
    # Start point
    start = GPSCoordinate(latitude=41.159, longitude=-8.629, altitude=10.0)
    scenario.add_start_point(start)
    
    # Goal: main building rooftop
    goal = GPSCoordinate(latitude=41.160, longitude=-8.631, altitude=10.0)
    scenario.add_goal_point(goal)
    
    # Obstacles: trees and structures (circular regions approximated as points)
    obstacle1 = GPSCoordinate(latitude=41.1592, longitude=-8.6295, altitude=10.0)
    scenario.add_obstacle(obstacle1, radius_m=30.0)
    
    obstacle2 = GPSCoordinate(latitude=41.1595, longitude=-8.6305, altitude=10.0)
    scenario.add_obstacle(obstacle2, radius_m=25.0)
    
    return scenario

def create_agricultural_survey_scenario() -> ScenarioConfiguration:
    """
    Scenario: Agricultural field survey with obstacle avoidance.
    
    Flying a larger quadcopter to survey a large agricultural area
    with trees and irrigation structures.
    
    Location: Rural area near Porto
    Start: Field entrance (41.15, -8.70, 50m)
    Goal: Far corner (41.14, -8.68, 50m)
    Grid: Larger cells (20m) for wider area coverage
    """
    scenario = ScenarioConfiguration(
        name="Agricultural Survey",
        drone_profile=DRONE_PROFILES['quadcopter_heavy'],
        grid_size_m=20.0  # 20m per grid cell for larger area
    )
    
    origin = GPSCoordinate(latitude=41.15, longitude=-8.70, altitude=50.0)
    scenario.set_origin(origin)
    
    start = GPSCoordinate(latitude=41.15, longitude=-8.70, altitude=55.0)
    scenario.add_start_point(start)
    
    goal = GPSCoordinate(latitude=41.14, longitude=-8.68, altitude=55.0)
    scenario.add_goal_point(goal)
    
    # Add tree clusters as obstacles
    for i in range(4):
        obs = GPSCoordinate(
            latitude=41.145 + i * 0.005,
            longitude=-8.695 + i * 0.003,
            altitude=45.0
        )
        scenario.add_obstacle(obs, radius_m=50.0)
    
    return scenario


def create_urban_delivery_scenario() -> ScenarioConfiguration:
    """
    Scenario: Urban last-mile delivery with building avoidance.
    
    Flying a lightweight quadcopter in an urban environment to deliver
    a package. Must avoid buildings, power lines, and other structures.
    
    Location: Urban area (Porto)
    Start: Warehouse rooftop (41.16, -8.62, 35m)
    Goal: Customer location (41.165, -8.625, 25m)
    """
    scenario = ScenarioConfiguration(
        name="Urban Delivery",
        drone_profile=DRONE_PROFILES['quadcopter_light'],
        grid_size_m=5.0  # Finer resolution for urban environment
    )
    
    origin = GPSCoordinate(latitude=41.16, longitude=-8.62, altitude=20.0)
    scenario.set_origin(origin)
    
    start = GPSCoordinate(latitude=41.16, longitude=-8.62, altitude=35.0)
    scenario.add_start_point(start)
    
    goal = GPSCoordinate(latitude=41.165, longitude=-8.625, altitude=25.0)
    scenario.add_goal_point(goal)
    
    # Buildings and structures (as obstacles)
    buildings = [
        GPSCoordinate(latitude=41.161, longitude=-8.618, altitude=30.0),
        GPSCoordinate(latitude=41.162, longitude=-8.623, altitude=28.0),
        GPSCoordinate(latitude=41.164, longitude=-8.627, altitude=35.0),
    ]
    
    for building in buildings:
        scenario.add_obstacle(building, radius_m=20.0)
    
    return scenario


def create_coastal_inspection_scenario() -> ScenarioConfiguration:
    """
    Scenario: Coastal infrastructure inspection using fixed-wing drone.
    
    Flying a fixed-wing UAV along coastline to inspect structures.
    Longer distance, higher efficiency priority.
    
    Location: Douro River mouth (Porto)
    Start: Beach launch point (41.13, -8.67, 10m)
    Goal: Harbor inspection point (41.12, -8.665, 15m)
    """
    scenario = ScenarioConfiguration(
        name="Coastal Inspection",
        drone_profile=DRONE_PROFILES['fixed_wing'],
        grid_size_m=25.0  # Larger cells for fixed-wing coverage
    )
    
    origin = GPSCoordinate(latitude=41.13, longitude=-8.67, altitude=5.0)
    scenario.set_origin(origin)
    
    start = GPSCoordinate(latitude=41.13, longitude=-8.67, altitude=10.0)
    scenario.add_start_point(start)
    
    goal = GPSCoordinate(latitude=41.12, longitude=-8.665, altitude=15.0)
    scenario.add_goal_point(goal)
    
    # Add some obstacles (rocky outcrops, buildings near coast)
    obstacles = [
        GPSCoordinate(latitude=41.1270, longitude=-8.6680, altitude=8.0),
        GPSCoordinate(latitude=41.1250, longitude=-8.6670, altitude=10.0),
    ]
    
    for obs in obstacles:
        scenario.add_obstacle(obs, radius_m=40.0)
    
    return scenario


# Dictionary for easy scenario selection
AVAILABLE_SCENARIOS = {
    'campus': create_campus_survey_scenario,
    'agriculture': create_agricultural_survey_scenario,
    'delivery': create_urban_delivery_scenario,
    'coastal': create_coastal_inspection_scenario,
}
