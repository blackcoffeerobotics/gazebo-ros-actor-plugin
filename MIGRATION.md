# Migration from ROS1 Noetic + Gazebo Classic to ROS2 Jazzy + Gazebo Harmonic

## Migration Status: IN PROGRESS

**Start Date:** October 18, 2025  
**Target Platform:** ROS2 Jazzy + Gazebo Harmonic  
**Current Platform:** ROS1 Noetic + Gazebo Classic 11

## References
1. [ROS1 to ROS2 plugin migration](https://gazebosim.org/docs/harmonic/migrating_gazebo_classic_ros2_packages/)
2. [Actor API migration from classic to ignition](https://gazebosim.org/api/sim/8/migrationactorapi.html)
3. [Gazebo FollowActor Plugin Example](https://github.com/gazebosim/gz-sim/tree/main/src/systems/follow_actor)

---

## Migration Strategy

### Phase 1: Build System (Tasks 1-3)
- Update package.xml to ROS2 format
- Convert CMakeLists.txt to ament_cmake
- Update dependencies

### Phase 2: Core Plugin Code (Tasks 4-8)
- Update header files with new APIs
- Convert plugin class structure to System interfaces
- Migrate ROS1 to ROS2 node/subscriptions
- Convert Gazebo Classic API to Harmonic ECS

### Phase 3: Configuration Files (Tasks 9-11)
- Convert XML launch to Python launch
- Update world files for Harmonic
- Migrate Python scripts to rclpy

### Phase 4: Integration & Testing (Tasks 12-15)
- Setup ROS-Gazebo bridge
- Update documentation
- Build and test

---

## Key API Changes

### ROS Migration
| ROS1 | ROS2 |
|------|------|
| `ros::NodeHandle` | `rclcpp::Node::SharedPtr` |
| `ros::Subscriber` | `rclcpp::Subscription<>::SharedPtr` |
| `geometry_msgs::Twist::ConstPtr` | `geometry_msgs::msg::Twist::SharedPtr` |
| `ros::init()` | `rclcpp::init()` |
| Custom callback queues | `rclcpp::executors::MultiThreadedExecutor` |

### Gazebo Migration
| Gazebo Classic | Gazebo Harmonic |
|----------------|-----------------|
| `ModelPlugin` | `gz::sim::System` + interfaces |
| `physics::ActorPtr` | `gz::sim::Entity` + components |
| `actor->WorldPose()` | `gz::sim::worldPose(entity, ecm)` |
| `actor->SetWorldPose()` | `ecm.SetComponentData<components::Pose>()` |
| `actor->SetScriptTime()` | `ecm.SetComponentData<components::AnimationTime>()` |
| `common::Time` | `std::chrono::duration` |
| `Load()` method | `Configure()` method |
| `OnUpdate()` | `PreUpdate()` / `PostUpdate()` |

### Actor Components in Harmonic
- `gz::sim::components::Actor` - Entity marker
- `gz::sim::components::Pose` - Position/orientation
- `gz::sim::components::AnimationName` - Animation identifier
- `gz::sim::components::AnimationTime` - Playback time
- `gz::sim::worldPose()` - Helper for world pose

---

## Estimated Timeline
- Build System: 1-2 hours
- Core Plugin Migration: 4-8 hours
- Configuration Files: 2-3 hours
- Integration & Testing: 2-4 hours
- **Total: 9-17 hours**

---

## Progress Log

### 2025-10-18
- Created migration plan and documentation
- Identified FollowActor plugin as reference implementation
- Confirmed actor animation support in Harmonic

---

## Known Issues & Solutions

### Issue: Actor Animation Control
**Status:** ✅ SOLVED  
**Solution:** Use `gz::sim::components::AnimationTime` component

### Issue: ROS-Gazebo Topic Bridge
**Status:** 📋 PLANNED  
**Solution:** Use `ros_gz_bridge` parameter_bridge node

---

## Testing Checklist
- [ ] Package compiles without errors
- [ ] Plugin loads in Gazebo (check verbose output)
- [ ] Actor appears in simulation
- [ ] Plugin responds to Gazebo topics
- [ ] ROS2 topics are visible
- [ ] Bridge forwards messages correctly
- [ ] Velocity control works (teleop_twist_keyboard)
- [ ] Path following works (path_publisher.py)
- [ ] Animation syncs with movement
- [ ] No memory leaks or crashes

---

## Rollback Plan
Keep original ROS1 version in `ros1-noetic` branch before starting migration.

```bash
git checkout -b ros1-noetic
git push origin ros1-noetic
git checkout master
```
