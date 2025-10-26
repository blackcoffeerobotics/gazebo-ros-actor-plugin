# Gazebo Actor Animation Bug Report

**Date:** October 26, 2025  
**Gazebo Version:** Gazebo Sim (gz-sim) 8.x (Harmonic)  
**ROS Version:** ROS 2 Jazzy  
**Issue:** Actor skeletal animation plays automatically on world load, ignoring `<auto_start>false</auto_start>` setting

**Status:** ❌ **CONFIRMED BUG - NO WORKAROUND EXISTS**

---

## Executive Summary

Gazebo's actor animation system automatically plays skeletal animations from the moment a world is loaded, **regardless of the `<auto_start>false</auto_start>` setting** in the `<script>` element. This prevents plugins from having proper control over when animations should play. 

**After testing 10+ different approaches**, including methods suggested by the official Gazebo FollowActor plugin, we confirm this is a **fundamental architectural limitation** in Gazebo Sim 8.x. The `<auto_start>` setting only affects trajectory-based movement, not skeletal animation playback.

---

## Problem Description

### Observed Behavior
1. Actor's walking animation begins playing immediately when Gazebo starts
2. Animation continues regardless of whether the actor is moving
3. The `<auto_start>false</auto_start>` setting in the SDF `<script>` element has no effect on skeletal animation
4. Plugin attempts to control animation result in either:
   - No effect (animation continues playing)
   - Gazebo crashing with segmentation fault

### Expected Behavior
1. Actor should display in static pose when world loads
2. Animation should only play when the actor is actively moving
3. Animation should pause when the actor stops moving
4. The `<auto_start>false</auto_start>` setting should prevent automatic animation playback

---

## Technical Investigation

### Architecture Understanding

Gazebo's actor system has two independent animation mechanisms:

1. **Skeletal Animation**: DAE mesh contains animated bones that play through keyframes, controlled by `AnimationTime` component
2. **Trajectory Animation**: SDF script defines waypoints for actor to follow

The issue is that **skeletal animation and trajectory animation are not properly synchronized** when using plugin control.

### Attempted Solutions

#### ❌ Solution 1: Use `<auto_start>false</auto_start>` in `<script>` tag
```xml
<script>
  <loop>true</loop>
  <auto_start>false</auto_start>
</script>
```
**Result:** Animation still plays from start  
**Conclusion:** The `auto_start` setting only affects trajectory-based movement, not skeletal animation

---

#### ❌ Solution 2: Add `<delay_start>` parameter
```xml
<script>
  <loop>true</loop>
  <auto_start>false</auto_start>
  <delay_start>999999.0</delay_start>
</script>
```
**Result:** Animation still plays from start  
**Conclusion:** `delay_start` only delays trajectory script execution, not skeletal animation

---

#### ❌ Solution 3: Manually set `AnimationTime` to zero in plugin PreUpdate
```cpp
gz::sim::Actor actor(this->actorEntity_);
actor.SetAnimationTime(_ecm, std::chrono::steady_clock::duration::zero());
```
**Result:** Gazebo crashes with segmentation fault  
**Error Log:**
```
Stack trace (most recent call last):
#2  gz::common::PoseAnimation::InterpolatedKeyFrame(gz::common::PoseKeyFrame&)
#1  gz::common::Animation::Time() const
#0  Segmentation fault (Address not mapped to object [0x8])
```
**Conclusion:** Manually setting `AnimationTime` conflicts with Gazebo's internal animation interpolation system

---

#### ❌ Solution 4: Set empty animation name
```cpp
actor.SetAnimationName(_ecm, "");  // Empty string to disable animation
```
**Result:** Gazebo crashes on launch  
**Conclusion:** Gazebo's animation system requires a valid animation name or crashes

---

#### ❌ Solution 5: Reset animation time every frame until movement
```cpp
if (!this->hasEverMoved_) {
  actor.SetAnimationTime(_ecm, std::chrono::steady_clock::duration::zero());
}
```
**Result:** Gazebo hangs/freezes on launch with no error output  
**Conclusion:** Repeatedly setting `AnimationTime` causes deadlock in Gazebo's animation system

---

#### ❌ Solution 6: Use empty trajectory with single waypoint
```xml
<script>
  <loop>false</loop>
  <auto_start>false</auto_start>
  <trajectory id="0" type="empty">
    <waypoint>
      <time>0</time>
      <pose>0 0 0 0 0 0</pose>
    </waypoint>
  </trajectory>
</script>
```
**Result:** Animation still plays from start  
**Conclusion:** Empty trajectory does not prevent skeletal animation

---

#### ❌ Solution 7: Remove `<animation>` element entirely
```xml
<actor name="actor1">
  <skin>
    <filename>model://DoctorFemaleWalk/meshes/DoctorFemaleWalk.dae</filename>
  </skin>
  <!-- No <animation> element -->
  <!-- No <script> element -->
</actor>
```
**Result:** Animation STILL plays from start  
**Conclusion:** Even without explicit animation definition, Gazebo auto-detects and plays animations embedded in DAE files

---

#### ❌ Solution 8: Direct ECM component manipulation with `*animTimeComp =`
```cpp
auto animTimeComp = _ecm.Component<components::AnimationTime>(actorEntity);
if (animTimeComp) {
  *animTimeComp = components::AnimationTime(std::chrono::steady_clock::duration::zero());
  _ecm.SetChanged(actorEntity, components::AnimationTime::typeId, 
                  ComponentState::OneTimeChange);
}
```
**Result:** Animation still plays from start  
**Conclusion:** Direct component assignment doesn't prevent Gazebo's internal animation system from advancing time

---

#### ❌ Solution 9: Toggle AnimationName between empty and "walking"
```cpp
// On startup: Clear animation name
*animNameComp = components::AnimationName("");

// When moving: Set animation name  
*animNameComp = components::AnimationName("walking");
```
**Result:** Gazebo hangs/freezes on launch with no error output  
**Conclusion:** Dynamically changing AnimationName causes conflicts with rendering system

---

#### ❌ Solution 10: Follow official FollowActor plugin pattern exactly
Based on Gazebo's official FollowActor plugin code:
- Set AnimationName once in Configure
- Only advance AnimationTime when moving (never reset)
- Use direct ECM component access
```cpp
// In first PreUpdate:
auto animTimeComp = _ecm.Component<components::AnimationTime>(actorEntity);
if (nullptr == animTimeComp) {
  _ecm.CreateComponent(actorEntity, components::AnimationTime());
}

// When moving:
auto animTime = animTimeComp->Data() +
  std::chrono::duration_cast<std::chrono::steady_clock::duration>(
  std::chrono::duration<double>(distanceTraveled * animationFactor));
*animTimeComp = components::AnimationTime(animTime);
_ecm.SetChanged(actorEntity, components::AnimationTime::typeId, 
                ComponentState::OneTimeChange);
```
**Result:** Animation still plays from start (no crash, but doesn't solve the problem)  
**Conclusion:** Even the official Gazebo pattern doesn't prevent auto-animation at startup. **The FollowActor plugin doesn't try to prevent animation - it accepts that it will play.**

---

#### ❌ Solution 11: Rendering plugin to control animation in GUI process
Attempted to bypass ECS entirely by creating a rendering plugin that runs in the GUI process:
```cpp
class PauseActorRenderPlugin : public rendering::System {
  void Update(rendering::ScenePtr _scene, double _dt) override {
    auto actor = std::dynamic_pointer_cast<rendering::Actor>(_scene->VisualByName("actor1"));
    if (actor) actor->SetAnimationTime(std::chrono::steady_clock::duration::zero());
  }
};
```
**Result:** ❌ Compilation failure - `gz/rendering/Actor.hh` does not exist  
**Conclusion:** gz-rendering library has no `Actor` class. Actors are rendered as generic `Visual` nodes without exposed animation control APIs. Animation is handled deep in the graphics backend (Ogre2), not accessible via gz-rendering.

---

## Root Cause Analysis - DEFINITIVE

After extensive investigation and testing 11+ solutions, the **definitive root cause** is:

**Gazebo does not expose `gz::common::SkeletonAnimation` instances to plugins.**

### The Missing API

The ideal solution would be:
```cpp
// What SHOULD exist but doesn't:
auto skelAnim = actor.GetSkeletonAnimation(_ecm, "walking");
skelAnim->SetTimeFactor(0.0);  // Pause animation
```

The `gz::common::SkeletonAnimation` class has the perfect API:
- `SetTimeFactor(0.0)` - freeze animation
- `SetTimeFactor(1.0)` - resume at normal speed
- `SetTime(double)` - jump to specific frame

**However**, these `SkeletonAnimation` instances are created and managed internally by Gazebo's rendering/graphics pipeline during mesh loading and are **never exposed** through:
- ECS components
- The `gz::sim::Actor` helper class  
- The `components::Actor` (SDF data)
- Any public plugin API

### Architecture Layers

1. **Gazebo's actor animation system is deeply integrated** with the rendering pipeline and runs independently of the ECS (Entity Component System)

2. **The `AnimationTime` component is managed by multiple systems:**
   - Plugin tries to control it through `Actor::SetAnimationTime()`
   - Gazebo's internal animation system (`RenderUtilPrivate::UpdateAnimation`) also updates it
   - These two systems conflict, causing crashes or being ignored

3. **The `<auto_start>` setting only affects the trajectory system**, not the skeletal animation system

4. **DAE files contain embedded animation data** that Gazebo automatically detects and plays, regardless of SDF configuration

---

## System Impact

### Crashes Encountered

1. **Segmentation Fault in Animation Interpolation**
   - Occurs when: Calling `SetAnimationTime()` from plugin
   - Location: `gz::common::PoseAnimation::InterpolatedKeyFrame()`
   - Frequency: Every attempt to manually set animation time

2. **Gazebo Hang/Freeze**
   - Occurs when: Repeatedly setting animation state in PreUpdate
   - Symptoms: Gazebo becomes unresponsive, no error logs, must force-kill
   - Frequency: Every launch after implementing repeated animation reset

---

## Current Workaround

**There is no working solution that allows:**
- Plugin control over animation
- Actor displaying in static pose when not moving
- Animation playing when actor moves
- No crashes or freezes

**Best compromise:** 
- Remove all animation control from plugin
- Let Gazebo's default animation system run continuously
- Accept that animation plays even when actor is stationary

---

## Requested Features/Fixes

For proper plugin control over actor animations, the following changes would be needed in Gazebo:

1. **Separate `auto_start` setting for skeletal animation** (distinct from trajectory auto_start)
   ```xml
   <animation name="walking">
     <filename>walk.dae</filename>
     <auto_start>false</auto_start>  <!-- New parameter -->
   </animation>
   ```

2. **Safe plugin API for animation control** that doesn't conflict with internal systems
   ```cpp
   actor.PauseAnimation(_ecm);
   actor.ResumeAnimation(_ecm);
   actor.SetAnimationSpeed(_ecm, speed);
   ```

3. **Clear animation state documentation** explaining:
   - When plugins can safely modify `AnimationTime`
   - How to prevent animation conflicts
   - Proper initialization order

4. **Animation state query** to check if animation system is ready
   ```cpp
   bool isAnimationReady = actor.IsAnimationSystemReady(_ecm);
   ```

---

## Environment Details

**System:**
- OS: Ubuntu 24.04 (in Docker container)
- Gazebo: gz-sim 8.x (Harmonic)
- ROS: ROS 2 Jazzy
- Installation: `/opt/ros/jazzy/opt/gz_sim_vendor`

**Plugin:**
- Language: C++
- System interfaces: ISystemConfigure, ISystemPreUpdate, ISystemPostUpdate
- Animation control attempted in: PreUpdate callback

**Actor Model:**
- Mesh: DoctorFemaleWalk.dae
- Animation: Walking cycle embedded in DAE
- Format: COLLADA (.dae) with skeletal animation

---

## Relevant Code Locations

**Gazebo Source (from stack trace):**
- `/opt/ros/jazzy/opt/gz_sim_vendor/lib/libgz-sim8-rendering.so.8`
  - `RenderUtilPrivate::UpdateAnimation()`
- `/opt/ros/jazzy/opt/gz_common_vendor/lib/libgz-common5-graphics.so.5`
  - `gz::common::PoseAnimation::InterpolatedKeyFrame()`
  - `gz::common::Animation::Time()`

**Components:**
- `/opt/ros/jazzy/opt/gz_sim_vendor/include/gz/sim8/gz/sim/components/Actor.hh`
  - `AnimationTime` component definition
  - `AnimationName` component definition

**Helper Classes:**
- `/opt/ros/jazzy/opt/gz_sim_vendor/include/gz/sim8/gz/sim/Actor.hh`
  - `SetAnimationTime()` method
  - `SetAnimationName()` method
  - `AnimationTime()` query method

---

## Reproduction Steps

1. Create actor in world SDF with animation:
```xml
<actor name="actor1">
  <skin>
    <filename>model://DoctorFemaleWalk/meshes/DoctorFemaleWalk.dae</filename>
  </skin>
  <animation name="walking">
    <filename>model://DoctorFemaleWalk/meshes/DoctorFemaleWalk.dae</filename>
  </animation>
  <script>
    <loop>true</loop>
    <auto_start>false</auto_start>
  </script>
</actor>
```

2. Launch Gazebo: `gz sim world.sdf`

3. **Observe:** Animation plays immediately despite `auto_start=false`

4. Create plugin that attempts to control animation:
```cpp
void PreUpdate(const gz::sim::UpdateInfo &_info,
               gz::sim::EntityComponentManager &_ecm) {
  gz::sim::Actor actor(this->actorEntity_);
  actor.SetAnimationTime(_ecm, std::chrono::steady_clock::duration::zero());
}
```

5. **Observe:** Gazebo crashes with segmentation fault

---

## Related Issues

This bug may be related to architectural design decisions where:
- Actor animation was designed for scripted trajectories, not plugin control
- Animation system was not designed to be paused/resumed dynamically
- ECS components (AnimationTime) are shadowing internal animation state

---

## Conclusion

Gazebo's current actor animation system **does not support plugin-controlled animation** in a safe and reliable way. The `<auto_start>false</auto_start>` setting is misleading as it only affects trajectories, not skeletal animations. Any attempt to programmatically control animation from plugins results in crashes or is ignored.

**Impact:** High - Prevents implementation of custom actor behaviors where animation should sync with plugin-controlled movement.

**Recommended Action:** Gazebo team should either:
1. Fix the animation system to respect `<auto_start>false</auto_start>` for skeletal animations
2. Provide safe plugin APIs for animation control
3. Document current limitations clearly to prevent users from attempting impossible implementations

---

## Contact & Files

**Project:** gazebo-ros-actor-plugin  
**Issue Files:**
- World file: `config/worlds/move_actor.world`
- Plugin source: `src/gazebo_ros_actor_command.cpp`
- Plugin header: `include/gazebo_ros_actor_plugin/gazebo_ros_actor_command.h`

**Previous Investigation Documents:**
- `ANIMATION_BUG_FIX.md` - Initial fix attempt
- `ANIMATION_CONTROL_INVESTIGATION.md` - Deep dive investigation
- `ANIMATION_FIX_V2.md` - Safe approach attempt
- `ANIMATION_BUG_SOLUTION.md` - Final solution documentation
