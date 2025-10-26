# Actor Animation Bug - Summary

**Problem:** Actor skeletal animation plays automatically from world startup, regardless of `<auto_start>false</auto_start>` setting.

**Tested 11+ Solutions:**
- SDF config (`auto_start`, `delay_start`, empty trajectories) - Animation still plays
- ECS manipulation (`AnimationTime`, `AnimationName` via Actor helper or direct component access) - Either ignored or causes crashes/hangs
- Official FollowActor pattern (only advance time when moving) - No crash, but animation still plays from start
- Rendering plugin approach - gz-rendering has no Actor class; animation control not exposed

**Root Cause:** The `gz::common::SkeletonAnimation` instances (which have `SetTimeFactor()` to pause/resume) are created internally during mesh loading and **never exposed to plugins** through any public API. The `<auto_start>` setting only affects trajectory movement, not skeletal animation.

**Status:** **Unfixable with current Gazebo Sim 8.x architecture.** Requires Gazebo team to expose SkeletonAnimation instances via new ECS component or API method.

**Workaround:** Accept that animation will play continuously, or use static mesh models instead of actors.
