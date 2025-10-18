# Quick Reference: ROS2 Migration

## What Changed?

### Package Structure
```
ROS1 Noetic + Gazebo Classic  →  ROS2 Jazzy + Gazebo Harmonic
catkin                        →  ament_cmake
package.xml (format 2)        →  package.xml (format 3)
.launch (XML)                 →  .launch.py (Python)
```

### Core APIs

#### ROS
```cpp
// OLD (ROS1)
ros::NodeHandle *node_;
ros::Subscriber sub_;
ros::init();

// NEW (ROS2)
rclcpp::Node::SharedPtr node_;
rclcpp::Subscription<...>::SharedPtr sub_;
rclcpp::init();
```

#### Gazebo
```cpp
// OLD (Classic)
physics::ActorPtr actor_;
actor_->WorldPose();
actor_->SetWorldPose(pose);

// NEW (Harmonic)
gz::sim::Entity actorEntity_;
gz::sim::worldPose(actorEntity_, ecm);
ecm.SetComponentData<components::Pose>(actorEntity_, pose);
```

#### Plugin Registration
```cpp
// OLD
GZ_REGISTER_MODEL_PLUGIN(GazeboRosActorCommand)

// NEW
GZ_ADD_PLUGIN(
    gazebo_ros_actor_plugin::GazeboRosActorCommand,
    gz::sim::System,
    GazeboRosActorCommand::ISystemConfigure,
    GazeboRosActorCommand::ISystemPreUpdate,
    GazeboRosActorCommand::ISystemPostUpdate)
```

## Build Commands

### ROS1 (Old)
```bash
catkin_make
source devel/setup.bash
roslaunch gazebo_ros_actor_plugin sim.launch
```

### ROS2 (New)
```bash
colcon build --packages-select gazebo_ros_actor_plugin
source install/setup.bash
ros2 launch gazebo_ros_actor_plugin sim.launch.py
```

## Testing Commands

### Check Topics
```bash
# OLD
rostopic list
rostopic echo /cmd_vel

# NEW
ros2 topic list
ros2 topic echo /cmd_vel
```

### Control Actor
```bash
# OLD
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

# NEW
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Publish Path
```bash
# OLD
rosrun gazebo_ros_actor_plugin path_publisher.py

# NEW  
ros2 run gazebo_ros_actor_plugin path_publisher.py
```

## Files Modified

1. ✅ `package.xml` - ROS2 format, dependencies
2. ✅ `CMakeLists.txt` - ament_cmake build
3. ✅ `include/gazebo_ros_actor_plugin/gazebo_ros_actor_command.h` - ROS2/Harmonic headers
4. ✅ `src/gazebo_ros_actor_command.cpp` - Complete rewrite
5. ✅ `launch/sim.launch.py` - Python launch (NEW)
6. ✅ `config/worlds/move_actor.world` - SDF 1.8, updated plugin syntax
7. ✅ `scripts/path_publisher.py` - rclpy
8. ✅ `README.md` - Updated documentation

## Backup Files Created

- `src/gazebo_ros_actor_command.cpp.old`
- `README.md.old`

Original launch file (`sim.launch`) still exists for reference.

## Next Steps

1. **Setup Environment:**
   ```bash
   # Install ROS2 Jazzy + Gazebo Harmonic
   sudo apt install ros-jazzy-desktop gz-harmonic
   sudo apt install ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge
   ```

2. **Build:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select gazebo_ros_actor_plugin
   ```

3. **Test:**
   ```bash
   source install/setup.bash
   ros2 launch gazebo_ros_actor_plugin sim.launch.py verbose:=true
   ```

4. **Verify:**
   - Plugin loads successfully
   - Actor appears in scene
   - Topics are available
   - Actor responds to commands

## Troubleshooting

**Plugin not found:**
```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ros2_ws/install/gazebo_ros_actor_plugin/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
```

**Topics not visible:**
```bash
ros2 node list  # Check if ros_gz_bridge is running
ros2 topic list # Check ROS topics
gz topic -l     # Check Gazebo topics
```

**Compilation errors:**
```bash
# Make sure all dependencies are installed
rosdep install --from-paths src --ignore-src -r -y

# Clean build
rm -rf build install log
colcon build --packages-select gazebo_ros_actor_plugin
```

## Documentation

- Full migration details: `MIGRATION.md`
- Summary and checklist: `MIGRATION_SUMMARY.md`
- Usage instructions: `README.md`
- This quick reference: `QUICK_REFERENCE.md`

---

**Migration Complete:** Ready for testing in ROS2 Jazzy + Gazebo Harmonic environment
