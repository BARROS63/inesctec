# Real-World Scenario Planner

A comprehensive system for planning UAV missions in real-world coordinates with adaptive spline selection and realistic drone energy modeling.

## Quick Start

```bash
# Run campus survey in sport mode (fastest)
python3 scenario_runner.py --scenario campus --mode sport

# Run urban delivery in efficient mode (energy-optimized)
python3 scenario_runner.py --scenario delivery --mode efficient

# Save mission file for flight controller
python3 scenario_runner.py --scenario campus --mode sport --save-mission mission.json
```

## Features

### Real-World Coordinates
- **GPS Input**: Define start point, goal, and obstacles in latitude/longitude
- **Altitude Support**: Plan multi-level missions with altitude profiles
- **Automatic Grid Conversion**: Converts real-world GPS to computational grid
- **Mission Export**: Generate JSON mission files with GPS waypoints

### Three Optimization Modes

#### Sport Mode (Fastest)
- Optimizes for **shortest flight time**
- Uses 85% of maximum drone speed
- Accepts aggressive maneuvers
- **Best for**: Time-critical deliveries, surveillance missions
- **Characteristics**: Highest power consumption, minimal flight time

#### Normal Mode (Balanced)
- Optimizes for **balanced efficiency and speed**
- Uses cruise speed (optimal efficiency-speed tradeoff)
- Moderate maneuver aggressiveness
- **Best for**: General missions, mixed priorities
- **Characteristics**: Medium power consumption and flight time

#### Efficient Mode (Energy-Optimized)
- Optimizes for **minimum energy consumption**
- Uses 60% of cruise speed for max efficiency
- Prefers smooth paths with minimal turns
- **Best for**: Long-range missions, battery-limited drones
- **Characteristics**: Lowest power consumption, longest flight time

### Drone Profiles

Built-in drone models with realistic energy calculations:

```python
'quadcopter_light':   # 0.5 kg, 1000 Wh battery, 15 m/s max
'quadcopter_medium':  # 2.0 kg, 5000 Wh battery, 20 m/s max (default)
'quadcopter_heavy':   # 5.0 kg, 15000 Wh battery, 18 m/s max
'fixed_wing':         # 1.5 kg, 3000 Wh battery, 25 m/s max, no hovering
```

### Predefined Scenarios

#### Campus Survey
- Location: INESC TEC, Porto
- Scenario: Building survey with obstacle avoidance
- Quadcopter: Medium (2 kg)
- Grid size: 10m cells
- Obstacles: Trees and structures

#### Agricultural Survey
- Location: Rural area near Porto
- Scenario: Large-area field inspection
- Quadcopter: Heavy (5 kg)
- Grid size: 20m cells
- Obstacles: Multiple tree clusters

#### Urban Delivery
- Location: Urban area in Porto
- Scenario: Last-mile package delivery
- Quadcopter: Light (0.5 kg)
- Grid size: 5m cells (fine resolution)
- Obstacles: Buildings in flight path

#### Coastal Inspection
- Location: Douro River mouth, Porto
- Scenario: Infrastructure inspection along coast
- Fixed-wing: 1.5 kg
- Grid size: 25m cells
- Obstacles: Rocky outcrops and coastal structures

## Usage Examples

### Basic Usage

```bash
# See all available options
python3 scenario_runner.py --help

# Run predefined scenario
python3 scenario_runner.py --scenario campus --mode sport

# Use different drone
python3 scenario_runner.py --scenario delivery --mode efficient --drone quadcopter_light
```

### Save Mission File

```bash
# Export GPS waypoints for flight controller
python3 scenario_runner.py --scenario campus --mode sport --save-mission campus_sport.json

# Mission file includes:
# - 201 GPS waypoints with altitude
# - Energy analysis for all modes
# - Scenario metadata
# - Selected spline type
```

### Compare Modes

```bash
# Sport mode - fastest
python3 scenario_runner.py --scenario delivery --mode sport

# Normal mode - balanced
python3 scenario_runner.py --scenario delivery --mode normal

# Efficient mode - energy-optimized
python3 scenario_runner.py --scenario delivery --mode efficient
```

## Understanding Output

### Console Output Example

```
================================================================================
Real-World Scenario: Campus Survey
Drone: Drone(m=2.0kg, v_cruise=12.0m/s, battery=5000.0Wh)
Mode: SPORT
================================================================================

Coordinate Conversion:
  Start (GPS): GPS(41.159000, -8.629000, 95.0m)
  Start (Grid): (0, 0)
  Goal (GPS): GPS(41.160000, -8.631000, 115.0m)
  Goal (Grid): (17, 11)
  Grid Size: 22 x 16
  Obstacles (grid): [(4, 2), (13, 6)]

Path Metrics:
  Raw Path            : Length=   9.153  Points=    3
  Bezier Spline       : Length=   9.198  Points=   61
  Cubic Spline        : Length=   9.219  Points=   61
  B-Spline            : Length=   9.010  Points=  201

Energy Consumption Analysis:
Mode            Energy (Wh)     Time (min)      Battery %
SPORT           1.48            0.20            0.0%
NORMAL          2.50            0.45            0.1%
EFFICIENT       2.62            0.80            0.1%
```

### Mission File Format

```json
{
  "metadata": {
    "scenario": "Campus Survey",
    "drone": "Drone(m=2.0kg, ...)",
    "mode": "efficient",
    "spline_used": "bspline"
  },
  "path_info": {
    "total_waypoints": 201,
    "path_length_m": 202.48,
    "energy_analysis": {
      "sport": {"energy_Wh": 1.48, "time_minutes": 0.20, ...},
      "normal": {"energy_Wh": 2.50, "time_minutes": 0.45, ...},
      "efficient": {"energy_Wh": 2.62, "time_minutes": 0.80, ...}
    }
  },
  "waypoints": [
    {
      "id": 0,
      "latitude": 41.159,
      "longitude": -8.629,
      "altitude": 95.0
    },
    ...
  ]
}
```

## Energy Model

### Sport Mode Calculation
```
speed_ms = max_speed * 0.85
horizontal_time = path_length / speed_ms
climb_rate = 3.0 m/s (aggressive)
thrust = mass * 9.81 N
propulsion_power = (thrust * speed) / motor_efficiency
maneuver_overhead = +10%
total_energy = power * time / 3600
```

### Normal Mode Calculation
```
speed_ms = cruise_speed  (optimal efficiency point)
climb_rate = 2.0 m/s (moderate)
curvature_penalty = 1.0 + (curvature - 1.0) * 0.05
total_energy = (propulsion_power + vertical_power) * penalty / 3600
```

### Efficient Mode Calculation
```
speed_ms = cruise_speed * 0.60  (maximum efficiency)
climb_rate = 1.0 m/s (gentle)
path_distance *= smoothness_factor
smoothness_bonus = max(0.8, 1.0 - (1.0 - smoothness) * 0.5)
total_energy = (power * time) * smoothness_bonus / 3600
```

## Spline Selection

The system automatically selects the best spline for each path:

- **Bezier**: Fast, smooth transitions (good for sport mode)
- **Cubic**: Natural curves, balanced (good for normal mode)
- **B-Spline**: Flexible control, smooth (good for efficient mode)

Selection criterion:
```
cost = length + gamma * curvature

Sport:     gamma = 0.1  (minimize curvature penalty)
Normal:    gamma = 0.5  (balanced)
Efficient: gamma = 1.0  (maximize smoothness preference)
```

## Creating Custom Scenarios

Add a new scenario to `lib/realworld.py`:

```python
def create_my_scenario():
    """My custom scenario."""
    scenario = ScenarioConfiguration(
        name="My Scenario",
        drone_profile=DRONE_PROFILES['quadcopter_medium'],
        grid_size_m=10.0
    )
    
    # Set origin point
    origin = GPSCoordinate(latitude=41.16, longitude=-8.63, altitude=50.0)
    scenario.set_origin(origin)
    
    # Define start and goal
    scenario.add_start_point(GPSCoordinate(
        latitude=41.160, longitude=-8.630, altitude=55.0
    ))
    scenario.add_goal_point(GPSCoordinate(
        latitude=41.165, longitude=-8.625, altitude=60.0
    ))
    
    # Add obstacles
    scenario.add_obstacle(GPSCoordinate(
        latitude=41.162, longitude=-8.627, altitude=50.0
    ), radius_m=30.0)
    
    return scenario

# Add to AVAILABLE_SCENARIOS dict
AVAILABLE_SCENARIOS['my_scenario'] = create_my_scenario
```

Then use:
```bash
python3 scenario_runner.py --scenario my_scenario --mode sport
```

## Output Files

Generated in `outputs/` directory:

- `raw_path.png` - Basic path visualization
- `adaptive_spline_comparison.png` - Overlay of all spline types
- `*_mission.json` - GPS waypoint files (when saved with `--save-mission`)

## Performance Characteristics

### Typical Execution Time
- Campus survey (small grid): ~2-3 seconds
- Delivery scenario (medium grid): ~5-8 seconds
- Large scenarios (>100 waypoints): 10-15 seconds

### Path Characteristics

| Scenario | Distance | Time | Waypoints |
|----------|----------|------|-----------|
| Campus   | 202m     | 0.2-0.8 min | 201 |
| Delivery | 696m     | 0.9-2.6 min | 201 |
| Coastal  | 2600m    | 2.0-8.0 min | 201 |

## Future Enhancements

- [ ] Interactive mission builder
- [ ] Real-time weather integration
- [ ] No-fly zone support
- [ ] Battery reserve calculations
- [ ] Multi-point missions
- [ ] Payload optimization
- [ ] Terrain elevation data
- [ ] Wind effect modeling

## Troubleshooting

### No path found
- Check that start and goal are not in obstacles
- Increase grid size or adjust obstacles
- Verify coordinates are within reasonable bounds

### High energy consumption
- Try efficient mode
- Reduce path curvature
- Use lighter drone profile
- Increase grid size

### Long mission time
- Use sport mode
- Switch to lighter drone
- Consider multiple smaller missions

## References

- **Pathfinding**: Lazy Theta* algorithm with line-of-sight shortcuts
- **Splines**: Bezier (Catmull-Rom), Natural cubic, B-spline (Cox-de Boor)
- **Energy**: Based on quadcopter dynamics and motor characteristics
- **Coordinates**: WGS-84 latitude/longitude to UTM-like grid conversion
