# Testing Checklist for ROS2 Migration

## Pre-Testing Setup

### Environment Setup
- [ ] ROS2 Jazzy installed
- [ ] Gazebo Harmonic (gz-sim8) installed
- [ ] ros_gz packages installed (`ros-jazzy-ros-gz`, `ros-jazzy-ros-gz-bridge`, `ros-jazzy-ros-gz-sim`)
- [ ] teleop_twist_keyboard installed (`ros-jazzy-teleop-twist-keyboard`)
- [ ] Workspace created (`~/ros2_ws`)

### Build Phase
- [ ] Package builds without errors: `colcon build --packages-select gazebo_ros_actor_plugin`
- [ ] No compiler warnings (or acceptable warnings documented)
- [ ] Library created: `install/gazebo_ros_actor_plugin/lib/libgazebo_ros_actor_plugin.so`
- [ ] Launch file installed: `install/gazebo_ros_actor_plugin/share/gazebo_ros_actor_plugin/launch/sim.launch.py`
- [ ] Script installed: `install/gazebo_ros_actor_plugin/lib/gazebo_ros_actor_plugin/path_publisher.py`
- [ ] Config files installed: `install/gazebo_ros_actor_plugin/share/gazebo_ros_actor_plugin/config/`

---

## Phase 1: Basic Launch Testing

### Gazebo Launch
- [ ] Source workspace: `source install/setup.bash`
- [ ] Launch succeeds: `ros2 launch gazebo_ros_actor_plugin sim.launch.py verbose:=true`
- [ ] Gazebo GUI opens (if not headless)
- [ ] No error messages in console
- [ ] World loads successfully

### Plugin Loading
- [ ] Plugin loading message appears in output:
  ```
  [Msg] GazeboRosActorCommand plugin attached to actor: actor1
  ```
- [ ] Configuration messages appear:
  ```
  [Msg] Actor control mode: velocity
  [Msg] Velocity topic: /cmd_vel
  [Msg] Path topic: /cmd_path
  [Msg] ROS2 node created: gazebo_actor_plugin_actor1
  [Msg] ROS2 subscriptions created and executor started
  ```
- [ ] No plugin errors or warnings
- [ ] Actor entity visible in Gazebo scene

### ROS2 Integration
- [ ] ROS2 node visible: `ros2 node list` shows node
- [ ] Topics available:
  ```bash
  ros2 topic list | grep cmd_vel
  ros2 topic list | grep cmd_path
  ```
- [ ] Bridge node running: `ros2 node list | grep ros_gz_bridge`
- [ ] Can echo topics: `ros2 topic echo /cmd_vel`

---

## Phase 2: Velocity Control Testing

### Setup
- [ ] Simulation running
- [ ] Open new terminal
- [ ] Source workspace: `source install/setup.bash`
- [ ] Launch teleop: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`

### Movement Tests
- [ ] Press 'i' → Actor moves forward
- [ ] Press 'k' → Actor stops
- [ ] Press 'j' → Actor rotates left
- [ ] Press 'l' → Actor rotates right
- [ ] Press ',' → Actor moves backward
- [ ] Movement is smooth (no jittering)
- [ ] Speed is reasonable (~1 m/s default)

### Animation Tests
- [ ] Walking animation plays when moving
- [ ] Animation speed matches movement speed
- [ ] Animation stops when actor stops
- [ ] No animation glitches or freezes
- [ ] Foot placement looks natural

### Edge Cases
- [ ] Rapid direction changes work
- [ ] Actor can move in all directions
- [ ] No crashes with extreme velocities
- [ ] Graceful handling of zero velocity
- [ ] Clean shutdown (Ctrl+C stops cleanly)

---

## Phase 3: Path Following Testing

### Setup
- [ ] Stop teleop (Ctrl+C)
- [ ] Edit world file: change `<follow_mode>velocity</follow_mode>` to `<follow_mode>path</follow_mode>`
- [ ] Restart Gazebo: `ros2 launch gazebo_ros_actor_plugin sim.launch.py`
- [ ] Launch path publisher: `ros2 run gazebo_ros_actor_plugin path_publisher.py`

### Path Tests
- [ ] Actor receives path:
  ```
  [INFO] [path_publisher_node]: Published path with 10 waypoints
  ```
- [ ] Actor starts moving toward first waypoint
- [ ] Actor rotates to face waypoint before moving
- [ ] Actor follows circular path
- [ ] Actor reaches each waypoint sequentially
- [ ] Actor stops at final waypoint
- [ ] Path can be republished to restart

### Behavior Tests
- [ ] Rotation tolerance is respected (~5°)
- [ ] Position tolerance is respected (0.1m)
- [ ] Actor doesn't overshoot waypoints
- [ ] Smooth transitions between waypoints
- [ ] Animation syncs with path movement

---

## Phase 4: Configuration Testing

### Parameter Tests
In world file, test different parameter values:

- [ ] `animation_factor` (try 2.0, 8.0)
  - Animation speed changes accordingly
  
- [ ] `linear_velocity` (try 0.5, 2.0)
  - Movement speed changes in path mode
  
- [ ] `angular_velocity` (try 1.0, 5.0)
  - Rotation speed changes
  
- [ ] `default_rotation` (try 0.0, 3.14)
  - Actor orientation adjusts

- [ ] Topic names (change to `/actor/cmd_vel`)
  - Actor subscribes to new topic
  - Teleop still works with new topic

### Mode Switching
- [ ] Switch from velocity to path mode (restart required)
- [ ] Switch from path to velocity mode (restart required)
- [ ] Both modes work correctly after switch

---

## Phase 5: Stability Testing

### Long-Running Tests
- [ ] Run simulation for 10+ minutes
- [ ] No memory leaks (monitor with `top` or `htop`)
- [ ] No performance degradation
- [ ] No accumulating errors in log
- [ ] Stable frame rate

### Stress Tests
- [ ] Rapid topic publishing (high frequency)
- [ ] Large paths (50+ waypoints)
- [ ] Continuous movement for extended time
- [ ] Multiple restart cycles
- [ ] No crashes or freezes

### Resource Usage
- [ ] CPU usage reasonable (< 100% for simulation)
- [ ] Memory usage stable (no growth over time)
- [ ] No zombie processes
- [ ] Clean shutdown releases resources

---

## Phase 6: Integration Testing

### Multi-Actor Test (Optional)
- [ ] Add second actor in world file
- [ ] Both actors load plugin successfully
- [ ] Each actor can be controlled independently
- [ ] No topic conflicts
- [ ] No performance issues

### Bridge Testing
- [ ] Messages flow ROS2 → Gazebo
- [ ] Topic remapping works if needed
- [ ] No message drops or delays
- [ ] Bridge handles high message rates
- [ ] Bridge reconnects if needed

### Gazebo Tools
- [ ] Can use gz command-line tools:
  ```bash
  gz topic -l
  gz topic -e -t /world/default/pose/info
  ```
- [ ] Actor pose visible in Gazebo topics
- [ ] Can monitor actor state from Gazebo

---

## Phase 7: Documentation Verification

### README
- [ ] Installation instructions work
- [ ] Build commands correct
- [ ] Launch commands correct
- [ ] Examples run successfully
- [ ] All links work
- [ ] Screenshots/GIFs still relevant

### Migration Docs
- [ ] MIGRATION.md is accurate
- [ ] API changes documented correctly
- [ ] QUICK_REFERENCE.md helpful
- [ ] MIGRATION_SUMMARY.md complete

### Code Documentation
- [ ] Header comments accurate
- [ ] Function descriptions correct
- [ ] Parameter descriptions match SDF
- [ ] No outdated comments

---

## Issue Tracking

### Known Issues
Document any issues found:

| Issue | Severity | Status | Notes |
|-------|----------|--------|-------|
| Example: Animation glitch at startup | Low | Open | Investigate AnimationTime component initialization |
| | | | |

### Blockers
Critical issues that prevent use:

- [ ] None identified (🎉)
- [ ] Document any blockers here

---

## Sign-Off

### Testing Complete
- [ ] All critical tests passed
- [ ] Known issues documented
- [ ] No blocking issues
- [ ] Ready for production use

### Tested By
- Name: _________________
- Date: _________________
- Environment: ROS2 _______ + Gazebo _______
- Notes: 

### Approval
- [ ] Code reviewer approval
- [ ] Integration testing passed
- [ ] Documentation approved
- [ ] Ready to merge/release

---

## Post-Testing Actions

- [ ] Update MIGRATION_SUMMARY.md with test results
- [ ] Document any workarounds needed
- [ ] Create GitHub issues for remaining bugs
- [ ] Update README with tested platforms
- [ ] Tag release if ready
- [ ] Announce migration completion

---

**Testing Status:** 🔴 Not Started | 🟡 In Progress | 🟢 Complete

**Last Updated:** October 18, 2025
