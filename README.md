# Duatic Simulation

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-blue.svg)](https://gazebosim.org/)
[![License](https://img.shields.io/badge/License-Duatic-blue.svg)](LICENSE)

Gazebo simulation environment for Duatic robotic platforms.

## Overview

Provides Gazebo Harmonic simulation worlds, robot spawning, and ROS-Gazebo bridge configuration for Duatic robots. Includes empty world and warehouse environment with AWS RoboMaker assets.

## Usage

### Launch Simulation Environment

#### Using ROS Node (Recommended)

The `start_sim` node provides a managed way to start and stop the simulation. It automatically ensures the simulation container is properly shut down when the node terminates.

```bash
# Empty world (default)
ros2 run duatic_simulation start_sim.py

# Warehouse world
ros2 run duatic_simulation start_sim.py --ros-args -p world:=warehouse

# Headless simulation
ros2 run duatic_simulation start_sim.py --ros-args -p world:=warehouse -p headless:=true
```

**Benefits:**
- Automatic cleanup: Docker container is stopped when node shuts down (Ctrl+C)
- ROS integration: Works as a standard ROS node with parameters
- Process management: Proper lifecycle handling

#### Using Launch File Directly

```bash
# Empty world (default)
ros2 launch duatic_simulation gazebo.launch.py

# Warehouse world
ros2 launch duatic_simulation gazebo.launch.py world:=warehouse

# Headless simulation
ros2 launch duatic_simulation gazebo.launch.py world:=warehouse headless:=true
```

### Spawn Robot

Prerequisite: URDF is being published with the topic /{namespace}/robot_description

```bash
# Basic spawn
ros2 launch duatic_simulation spawn.launch.py namespace:=robot1 x:=0 y:=0 z:=1 yaw:=0

# Multiple robots
ros2 launch duatic_simulation spawn.launch.py namespace:=alpha x:=0 y:=0 z:=1 yaw:=0 &
ros2 launch duatic_simulation spawn.launch.py namespace:=beta x:=2 y:=2 z:=1 yaw:=1.57 &
```

### ROS-Gazebo Bridge

```bash
# Launch bridge with config file
ros2 launch duatic_simulation ros_gz_bridge.launch.py \
  namespace:=robot1 \
  world:=warehouse \
  config_file:=/path/to/bridge_config.yaml
```

## Launch Files

| File | Description |
|------|-------------|
| `gazebo.launch.py` | Main simulation environment |
| `spawn.launch.py` | Robot spawning interface |
| `ros_gz_bridge.launch.py` | ROS-Gazebo topic bridge |

## Nodes

### start_sim.py

ROS 2 node that manages the simulation Docker container lifecycle.

**Parameters:**
- `world` (string, default: "empty"): World name to load
- `headless` (bool, default: false): Run simulation without GUI

**Features:**
- Starts simulation container using docker compose
- Automatically stops container on node shutdown
- Proper cleanup with Ctrl+C
- ROS parameter support

**Example:**
```bash
ros2 run duatic_simulation start_sim.py --ros-args -p world:=warehouse -p headless:=false
```

## Parameters

### gazebo.launch.py
| Parameter | Default | Description |
|-----------|---------|-------------|
| `world` | `"empty"` | World name (empty, warehouse) |
| `headless` | `false` | Run without GUI |

### spawn.launch.py
| Parameter | Default | Description |
|-----------|---------|-------------|
| `namespace` | `"empty_namespace"` | Robot namespace |
| `x`, `y`, `z` | `0.0` | Spawn position |
| `yaw` | `0.0` | Spawn orientation |

### ros_gz_bridge.launch.py
| Parameter | Default | Description |
|-----------|---------|-------------|
| `namespace` | `"empty_namespace"` | Robot namespace |
| `world` | `"undefined"` | World name |
| `config_file` | - | Bridge configuration file |

## Available Worlds

- **duatic_empty**: Minimal environment for basic testing
- **warehouse**: AWS RoboMaker warehouse with shelves, pallets, and clutter

## Assets

### AWS RoboMaker Models
- Warehouse infrastructure (shelves, walls, ground)
- Props (buckets, lamps, trash cans, pallet jacks)
- Clutter objects for realistic environments
