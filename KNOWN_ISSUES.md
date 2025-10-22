# Known Issues

## Animation Auto-Start Bug

**Status:** Open  
**Priority:** Low  
**Date:** 2025-10-22

### Description
The actor animation starts playing automatically when the world loads, even though we've tried multiple approaches to prevent this:
1. Added `<auto_start>false</auto_start>` to the `<script>` section in the world file
2. Reset `AnimationTime` to zero in the first `PreUpdate()` call
3. Only advance `AnimationTime` when the actor is moving (`distanceTraveled > 0.0001`)

### Current Behavior
- Actor spawns and immediately starts playing the walking animation
- Animation does properly pause when velocity is zero and advance when moving
- The issue is purely cosmetic - the actor animates in place until first velocity command

### Expected Behavior
- Actor should spawn in a static T-pose or first frame of animation
- Animation should only start when first velocity command is received

### Possible Causes
1. Gazebo Harmonic's actor system may have its own animation controller that runs before plugin updates
2. The `<auto_start>false</auto_start>` parameter may not be fully implemented in GZ Harmonic
3. AnimationTime component might be managed by multiple systems causing conflicts

### Workaround
None currently - the actor will animate in place on spawn until velocity commands are sent.

### Next Steps to Try
- Check if there's an `AnimationName` component that needs to be cleared
- Look into Gazebo source code to understand actor animation lifecycle
- Try using a different animation file or creating a custom idle animation
- Check if there's a way to disable the actor's built-in animation system entirely
