#include <gazebo_ros_actor_plugin/gazebo_ros_actor_command.h>

#include <ignition/plugin/Register.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/math/Angle.hh>

#include <cmath>
#include <functional>

#define WALKING_ANIMATION "walking"
// IGN_DTOR is already defined in ignition/math/Angle.hh

using namespace gazebo_ros_actor_plugin;

/////////////////////////////////////////////////
GazeboRosActorCommand::GazeboRosActorCommand()
: actorEntity_(ignition::gazebo::kNullEntity),
  animationFactor_(4.0),
  lastUpdate_(std::chrono::steady_clock::duration::zero()),
  followMode_("velocity"),
  targetVel_(ignition::math::Pose3d::Zero),
  linVelocity_(1.0),
  angVelocity_(IGN_DTOR(10)),
  idx_(0),
  linTolerance_(0.1),
  angTolerance_(IGN_DTOR(5)),
  defaultRotation_(M_PI/2),
  pathCompletedLogged_(false) {
}

/////////////////////////////////////////////////
GazeboRosActorCommand::~GazeboRosActorCommand() {
  // Nothing to clean up for GZ transport node
}

/////////////////////////////////////////////////
void GazeboRosActorCommand::Configure(
    const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &/*_eventMgr*/) {
  
  this->actorEntity_ = _entity;
  
  // Verify this is an actor
  if (!_ecm.Component<ignition::gazebo::components::Actor>(this->actorEntity_)) {
    ignerr << "GazeboRosActorCommand plugin must be attached to an actor entity.\n";
    return;
  }
  
  // Get actor name for logging
  auto nameComp = _ecm.Component<ignition::gazebo::components::Name>(this->actorEntity_);
  std::string actorName = nameComp ? nameComp->Data() : "unknown";
  ignmsg << "GazeboRosActorCommand attached to actor: " << actorName << std::endl;
  
  // Set default values for parameters
  this->followMode_ = "velocity";
  this->velTopic_ = "/cmd_vel";
  this->pathTopic_ = "/cmd_path";
  this->linTolerance_ = 0.1;
  this->linVelocity_ = 1.0;
  this->angTolerance_ = IGN_DTOR(5);
  this->angVelocity_ = IGN_DTOR(10);
  this->animationFactor_ = 4.0;
  this->defaultRotation_ = M_PI/2;
  
  // Override default parameter values with values from SDF
  if (_sdf->HasElement("follow_mode")) {
    this->followMode_ = _sdf->Get<std::string>("follow_mode");
  }
  if (_sdf->HasElement("vel_topic")) {
    this->velTopic_ = _sdf->Get<std::string>("vel_topic");
  }
  if (_sdf->HasElement("path_topic")) {
    this->pathTopic_ = _sdf->Get<std::string>("path_topic");
  }
  if (_sdf->HasElement("linear_tolerance")) {
    this->linTolerance_ = _sdf->Get<double>("linear_tolerance");
  }
  if (_sdf->HasElement("linear_velocity")) {
    this->linVelocity_ = _sdf->Get<double>("linear_velocity");
  }
  if (_sdf->HasElement("angular_tolerance")) {
    this->angTolerance_ = _sdf->Get<double>("angular_tolerance");
  }
  if (_sdf->HasElement("angular_velocity")) {
    this->angVelocity_ = _sdf->Get<double>("angular_velocity");
  }
  if (_sdf->HasElement("animation_factor")) {
    this->animationFactor_ = _sdf->Get<double>("animation_factor");
  }
  if (_sdf->HasElement("default_rotation")) {
    this->defaultRotation_ = _sdf->Get<double>("default_rotation");
  }
  
  // Subscribe to BOTH IGN topics (velocity and path) to allow dynamic mode switching
  if (!this->node_.Subscribe(this->velTopic_, &GazeboRosActorCommand::VelCallback, this)) {
    ignerr << "Failed to subscribe to velocity topic: " << this->velTopic_ << std::endl;
  }
  
  if (!this->node_.Subscribe(this->pathTopic_, &GazeboRosActorCommand::PathCallback, this)) {
    ignerr << "Failed to subscribe to path topic: " << this->pathTopic_ << std::endl;
  }
  
  // Don't initialize with a default waypoint - wait for actual path commands
  // this->targetPoses_.push_back(gz::math::Vector3d(0.0, 0.0, 0.0));
  // this->targetPose_ = this->targetPoses_.at(this->idx_);
  
  // Don't touch AnimationTime here - let PreUpdate handle it
  // The actor system will create it based on the SDF <auto_start> setting
  
  this->lastUpdate_ = std::chrono::steady_clock::duration::zero();
}

/////////////////////////////////////////////////
void GazeboRosActorCommand::VelCallback(const ignition::msgs::Twist &msg) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  ignition::math::Vector3d velCmd;
  velCmd.X() = msg.linear().x();
  velCmd.Z() = msg.angular().z();
  this->cmdQueue_.push(velCmd);
}

/////////////////////////////////////////////////
void GazeboRosActorCommand::PathCallback(const ignition::msgs::Pose_V &msg) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  
  // Extract poses from the Pose_V message
  std::vector<ignition::math::Vector3d> poses;
  
  for (int i = 0; i < msg.pose_size(); ++i) {
    const auto& pose = msg.pose(i);
    double x = pose.position().x();
    double y = pose.position().y();
    
    // Convert quaternion to yaw angle
    ignition::math::Quaterniond quat(
      pose.orientation().w(),
      pose.orientation().x(),
      pose.orientation().y(),
      pose.orientation().z()
    );
    double yaw = quat.Euler().Z();
    
    poses.push_back(ignition::math::Vector3d(x, y, yaw));
  }
  
  if (!poses.empty()) {
    this->pathQueue_.push(poses);
    ignmsg << "New path received with " << poses.size() << " waypoints" << std::endl;
  } else {
    ignwarn << "Received empty path" << std::endl;
  }
}

/////////////////////////////////////////////////
void GazeboRosActorCommand::PreUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm) {
  
  IGN_PROFILE("GazeboRosActorCommand::PreUpdate");
  
  // Initialize last update on first iteration and reset animation
  if (this->lastUpdate_ == std::chrono::steady_clock::duration::zero()) {
    this->lastUpdate_ = _info.simTime;
    
    // Reset animation time to zero on first update to stop auto-animation
    ignition::gazebo::Actor actor(this->actorEntity_);
    actor.SetAnimationTime(_ecm, std::chrono::steady_clock::duration::zero());
    
    return;
  }
  
  // Calculate time delta
  std::chrono::duration<double> dt = _info.simTime - this->lastUpdate_;
  
  // Get current actor pose
  auto currentPose = ignition::gazebo::worldPose(this->actorEntity_, _ecm);
  ignition::math::Vector3d rpy = currentPose.Rot().Euler();
  
  ignition::math::Pose3d newPose = currentPose;
  double distanceTraveled = 0.0;
  
  if (this->followMode_ == "path") {
    std::lock_guard<std::mutex> lock(this->mutex_);
    
    // Check if there's a new path command
    if (!this->pathQueue_.empty()) {
      // Get the new path and replace current target poses
      this->targetPoses_ = this->pathQueue_.front();
      this->pathQueue_.pop();
      
      // Reset index to start from the beginning of the new path
      this->idx_ = 0;
      if (!this->targetPoses_.empty()) {
        this->targetPose_ = this->targetPoses_.at(this->idx_);
        this->pathCompletedLogged_ = false; // Reset the flag for new path
        ignmsg << "New path loaded with " << this->targetPoses_.size() 
              << " waypoints" << std::endl;
      }
    }
    
    // Only proceed if we have valid target poses
    if (this->targetPoses_.empty() || this->idx_ >= static_cast<int>(this->targetPoses_.size())) {
      // Silently skip - no targets available yet
      this->lastUpdate_ = _info.simTime;
      return;
    }
    
    ignition::math::Vector2d targetPos2d(this->targetPose_.X(), this->targetPose_.Y());
    ignition::math::Vector2d currentPos2d(currentPose.Pos().X(), currentPose.Pos().Y());
    ignition::math::Vector2d pos = targetPos2d - currentPos2d;
    double distance = pos.Length();
    
    // Check if actor has reached current target position
    if (distance < this->linTolerance_) {
      // If there are more targets, choose new target
      if (this->idx_ < static_cast<int>(this->targetPoses_.size()) - 1) {
        this->ChooseNewTarget();
        pos.X() = this->targetPose_.X() - currentPose.Pos().X();
        pos.Y() = this->targetPose_.Y() - currentPose.Pos().Y();
      } else {
        // All targets have been accomplished, stop moving
        if (!this->pathCompletedLogged_) {
          ignmsg << "Path completed - all waypoints reached" << std::endl;
          this->pathCompletedLogged_ = true;
        }
        pos.X() = 0;
        pos.Y() = 0;
      }
    }
    
    // Normalize the direction vector
    if (pos.Length() != 0) {
      pos = pos / pos.Length();
    }
    
    // Calculate target yaw to face the waypoint
    // For path mode, we directly set the orientation without offset
    double targetYaw = std::atan2(pos.Y(), pos.X());
    
    // Always set orientation to face the direction of movement
    newPose.Rot() = ignition::math::Quaterniond(0, 0, targetYaw);
    
    // Move towards the target position
    if (pos.Length() != 0) {
      newPose.Pos().X() += pos.X() * this->linVelocity_ * dt.count();
      newPose.Pos().Y() += pos.Y() * this->linVelocity_ * dt.count();
      distanceTraveled = (pos * this->linVelocity_ * dt.count()).Length();
    }
    
  } else if (this->followMode_ == "velocity") {
    std::lock_guard<std::mutex> lock(this->mutex_);
    
    // Get velocity command from queue
    if (!this->cmdQueue_.empty()) {
      ignition::math::Vector3d vel = this->cmdQueue_.front();
      this->cmdQueue_.pop();
      
      this->targetVel_.Pos().X() = vel.X();
      this->targetVel_.Rot() = ignition::math::Quaterniond(0, 0, vel.Z());
    }
    
    // Apply velocity (only if non-zero)
    if (std::abs(this->targetVel_.Pos().X()) > 0.001 || 
        std::abs(this->targetVel_.Rot().Euler().Z()) > 0.001) {
      // Fixed: Don't subtract default_rotation from current yaw
      double dx = this->targetVel_.Pos().X() *
                  std::cos(currentPose.Rot().Euler().Z()) * dt.count();
      double dy = this->targetVel_.Pos().X() *
                  std::sin(currentPose.Rot().Euler().Z()) * dt.count();
      
      newPose.Pos().X() += dx;
      newPose.Pos().Y() += dy;
      
      double newYaw = rpy.Z() + this->targetVel_.Rot().Euler().Z() * dt.count();
      // Keep orientation upright - only set yaw, no roll/pitch
      newPose.Rot() = ignition::math::Quaterniond(0, 0, newYaw);
      
      distanceTraveled = std::sqrt(dx * dx + dy * dy);
    } else {
      this->targetVel_ = ignition::math::Pose3d::Zero;
    }
  }
  
  // Update actor pose - actors need special handling
  auto poseComp = _ecm.Component<ignition::gazebo::components::Pose>(this->actorEntity_);
  if (poseComp) {
    _ecm.SetComponentData<ignition::gazebo::components::Pose>(this->actorEntity_, newPose);
  } else {
    _ecm.CreateComponent(this->actorEntity_, ignition::gazebo::components::Pose(newPose));
  }
  
  // Mark pose as changed so the rendering updates
  _ecm.SetChanged(this->actorEntity_, ignition::gazebo::components::Pose::typeId, ignition::gazebo::ComponentState::OneTimeChange);
  
  // Update animation time based on movement
  if (distanceTraveled > 0.0001) {
    // Actor is moving - advance animation
    ignition::gazebo::Actor actor(this->actorEntity_);
    auto currentAnimTime = actor.AnimationTime(_ecm);
    if (currentAnimTime) {
      std::chrono::duration<double> animTimeDelta(distanceTraveled * this->animationFactor_);
      auto newAnimTime = *currentAnimTime + std::chrono::duration_cast<std::chrono::steady_clock::duration>(animTimeDelta);
      actor.SetAnimationTime(_ecm, newAnimTime);
    }
  }
  // When not moving, don't update animation time - it will stay at current frame (paused)
  
  this->lastUpdate_ = _info.simTime;
}

/////////////////////////////////////////////////
void GazeboRosActorCommand::PostUpdate(
    const ignition::gazebo::UpdateInfo &/*_info*/,
    const ignition::gazebo::EntityComponentManager &/*_ecm*/) {
  // Currently not used, but available for future extensions
  // Could be used for publishing actor state, etc.
}

/////////////////////////////////////////////////
void GazeboRosActorCommand::ChooseNewTarget() {
  this->idx_++;
  
  // Set next target
  if (this->idx_ < static_cast<int>(this->targetPoses_.size())) {
    this->targetPose_ = this->targetPoses_.at(this->idx_);
  }
}

// Register the plugin
IGNITION_ADD_PLUGIN(
    gazebo_ros_actor_plugin::GazeboRosActorCommand,
    ignition::gazebo::System,
    GazeboRosActorCommand::ISystemConfigure,
    GazeboRosActorCommand::ISystemPreUpdate,
    GazeboRosActorCommand::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
    gazebo_ros_actor_plugin::GazeboRosActorCommand,
    "gazebo_ros_actor_plugin::GazeboRosActorCommand")
