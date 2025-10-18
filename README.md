# Gazebo ROS2 Actor Plugin (Harmonic)

## About

The `gazebo_ros_actor_plugin` package contains a plugin for **Gazebo Harmonic** and **ROS2** that enables dynamic control of actors in simulation. The plugin allows you to control actors using either position or velocity commands.

**Migration Note:** This package has been migrated from ROS1 Noetic + Gazebo Classic 11 to ROS2 Jazzy + Gazebo Harmonic. See `MIGRATION.md` for details.

## System Requirements

Before using this package, make sure that you meet the following requirements:

- **ROS 2 Jazzy** (or later)
- **Gazebo Harmonic** (gz-sim8)
- **ros_gz** packages (`ros_gz_sim`, `ros_gz_bridge`)

## Installation

### Prerequisites

```bash
# Install ROS2 Jazzy (Ubuntu 24.04)
# Follow official instructions at: https://docs.ros.org/en/jazzy/Installation.html

# Install Gazebo Harmonic
sudo apt-get update
sudo apt-get install gz-harmonic

# Install ROS-Gazebo bridge packages
sudo apt-get install ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim
```

### Build from Source

```bash
# Create a ROS2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repository
git clone https://github.com/blackcoffeerobotics/gazebo-ros-actor-plugin.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build --packages-select gazebo_ros_actor_plugin

# Source the workspace
source ~/ros2_ws/install/setup.bash
```

## Usage

### Running the Plugin

1. **Source your workspace:**
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

2. **Configure the actor mode** in `config/worlds/move_actor.world`:
   - Edit the `follow_mode` parameter to either `velocity` or `path`

3. **Launch the simulation:**
   ```bash
   ros2 launch gazebo_ros_actor_plugin sim.launch.py
   ```

   Launch arguments:
   - `verbose:=true/false` - Enable verbose output (default: true)
   - `headless:=true/false` - Run without GUI (default: false)
   - `enable_bridge:=true/false` - Enable ROS-Gazebo bridge (default: true)

### Control Methods

#### 1. Velocity Control

```bash
# Install teleop keyboard
sudo apt-get install ros-jazzy-teleop-twist-keyboard

# Run teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use `i/j/k/l/,` keys to control movement.

![Velocity control](res/actor_vel.gif)

#### 2. Path Following

```bash
ros2 run gazebo_ros_actor_plugin path_publisher.py
```

Publishes a circular path with 10 waypoints.

![Path control](res/actor_path.gif)

## Configuration Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `follow_mode` | `velocity` | Control mode: `path` or `velocity` |
| `vel_topic` | `/cmd_vel` | Velocity command topic |
| `path_topic` | `/cmd_path` | Path command topic |
| `animation_factor` | `4.0` | Animation speed multiplier |
| `linear_velocity` | `1.0` | Movement speed (m/s) |
| `angular_velocity` | `2.5` | Rotation speed (rad/s) |
| `default_rotation` | `1.57` | Skin alignment offset (rad) |

## ROS2 API

### Subscribed Topics

- `/cmd_vel` (`geometry_msgs/msg/Twist`) - Velocity commands
- `/cmd_path` (`nav_msgs/msg/Path`) - Path waypoints

## Resources

- [Migration Guide](MIGRATION.md)
- [Gazebo Harmonic Docs](https://gazebosim.org/docs/harmonic)
- [ROS2 Jazzy Docs](https://docs.ros.org/en/jazzy/)
- [Demo Videos](https://youtube.com/playlist?list=PL_jbb--NzdcAPhl06Fey7m6UO2aNw8a8d)

## License

Apache-2.0

## Authors

- Animesh Singhal (animesh@blackcoffeerobotics.com)
- Gaurav Gupta (gaurav@blackcoffeerobotics.com)
