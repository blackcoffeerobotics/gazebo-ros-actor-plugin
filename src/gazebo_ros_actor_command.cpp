#include <gazebo_ros_actor_plugin/gazebo_ros_actor_command.h>

#include <gz/plugin/Register.hh>
#include <gz/common/Profiler.hh>
#include <gz/math/Angle.hh>

#include <cmath>
#include <functional>

#define WALKING_ANIMATION "walking"
// GZ_DTOR is already defined in gz/math/Angle.hh

using namespace gazebo_ros_actor_plugin;

/////////////////////////////////////////////////
GazeboRosActorCommand::GazeboRosActorCommand()
: actorEntity_(gz::sim::kNullEntity),
  animationFactor_(4.0),
  lastUpdate_(std::chrono::steady_clock::duration::zero()),
  followMode_("velocity"),
  linVelocity_(1.0),
  angVelocity_(GZ_DTOR(10)),
  idx_(0),
  linTolerance_(0.1),
  angTolerance_(GZ_DTOR(5)),
  defaultRotation_(M_PI/2) {
}

/////////////////////////////////////////////////
GazeboRosActorCommand::~GazeboRosActorCommand() {
  // Stop the executor
  if (this->executor_) {
    this->executor_->cancel();
  }
  
  // Join the executor thread
  if (this->executorThread_.joinable()) {
    this->executorThread_.join();
  }
  
  // Shutdown ROS node
  if (this->rosNode_) {
    this->rosNode_.reset();
  }
}

/////////////////////////////////////////////////
void GazeboRosActorCommand::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &/*_eventMgr*/) {
  
  this->actorEntity_ = _entity;
  
  // Verify this is an actor
  if (!_ecm.Component<gz::sim::components::Actor>(this->actorEntity_)) {
    gzerr << "GazeboRosActorCommand plugin must be attached to an actor entity.\n";
    return;
  }
  
  // Get actor name for logging
  auto nameComp = _ecm.Component<gz::sim::components::Name>(this->actorEntity_);
  std::string actorName = nameComp ? nameComp->Data() : "unknown";
  gzmsg << "GazeboRosActorCommand plugin attached to actor: " << actorName << std::endl;
  
  // Set default values for parameters
  this->followMode_ = "velocity";
  this->velTopic_ = "/cmd_vel";
  this->pathTopic_ = "/cmd_path";
  this->linTolerance_ = 0.1;
  this->linVelocity_ = 1.0;
  this->angTolerance_ = GZ_DTOR(5);
  this->angVelocity_ = GZ_DTOR(10);
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
  
  gzmsg << "Actor control mode: " << this->followMode_ << std::endl;
  gzmsg << "Velocity topic: " << this->velTopic_ << std::endl;
  gzmsg << "Path topic: " << this->pathTopic_ << std::endl;
  
  // Initialize ROS2 node
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  
  std::string nodeName = "gazebo_actor_plugin_" + actorName;
  this->rosNode_ = std::make_shared<rclcpp::Node>(nodeName);
  
  gzmsg << "ROS2 node created: " << nodeName << std::endl;
  
  // Create velocity subscriber
  this->velSub_ = this->rosNode_->create_subscription<geometry_msgs::msg::Twist>(
      this->velTopic_, 10,
      std::bind(&GazeboRosActorCommand::VelCallback, this, std::placeholders::_1));
  
  // Create path subscriber
  this->pathSub_ = this->rosNode_->create_subscription<nav_msgs::msg::Path>(
      this->pathTopic_, 10,
      std::bind(&GazeboRosActorCommand::PathCallback, this, std::placeholders::_1));
  
  // Create and start executor in a separate thread
  this->executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  this->executor_->add_node(this->rosNode_);
  this->executorThread_ = std::thread([this]() { this->executor_->spin(); });
  
  gzmsg << "ROS2 subscriptions created and executor started." << std::endl;
  
  // Initialize target poses with origin
  this->targetPoses_.push_back(gz::math::Vector3d(0.0, 0.0, 0.0));
  this->targetPose_ = this->targetPoses_.at(this->idx_);
  
  // Check if animation exists
  auto animNameComp = _ecm.Component<gz::sim::components::AnimationName>(this->actorEntity_);
  if (animNameComp) {
    gzmsg << "Actor has animation: " << animNameComp->Data() << std::endl;
  } else {
    gzwarn << "Actor does not have animation component." << std::endl;
  }
  
  // Create AnimationTime component if it doesn't exist
  if (!_ecm.Component<gz::sim::components::AnimationTime>(this->actorEntity_)) {
    _ecm.CreateComponent(this->actorEntity_,
        gz::sim::components::AnimationTime(std::chrono::steady_clock::duration::zero()));
    gzmsg << "Created AnimationTime component for actor." << std::endl;
  }
  
  this->lastUpdate_ = std::chrono::steady_clock::duration::zero();
  
  gzmsg << "GazeboRosActorCommand plugin configured successfully." << std::endl;
}

/////////////////////////////////////////////////
void GazeboRosActorCommand::VelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  gz::math::Vector3d velCmd;
  velCmd.X() = msg->linear.x;
  velCmd.Z() = msg->angular.z;
  this->cmdQueue_.push(velCmd);
}

/////////////////////////////////////////////////
void GazeboRosActorCommand::PathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  
  // Extract the poses from the Path message
  const std::vector<geometry_msgs::msg::PoseStamped>& poses = msg->poses;
  
  // Clear existing targets (except the first one which is origin)
  if (this->targetPoses_.size() > 1) {
    this->targetPoses_.erase(this->targetPoses_.begin() + 1, this->targetPoses_.end());
  }
  
  // Extract x, y, and yaw from each pose and store it as a target
  for (size_t i = 0; i < poses.size(); ++i) {
    const auto& pose = poses[i].pose;
    const double x = pose.position.x;
    const double y = pose.position.y;
    
    // Convert quaternion to yaw
    const auto& q = pose.orientation;
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    
    this->targetPoses_.push_back(gz::math::Vector3d(x, y, yaw));
  }
  
  // Reset to first target
  this->idx_ = 0;
  if (!this->targetPoses_.empty()) {
    this->targetPose_ = this->targetPoses_.at(this->idx_);
  }
  
  RCLCPP_INFO(this->rosNode_->get_logger(),
              "Received path with %zu waypoints", poses.size());
}

/////////////////////////////////////////////////
void GazeboRosActorCommand::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) {
  
  GZ_PROFILE("GazeboRosActorCommand::PreUpdate");
  
  // Initialize last update on first iteration
  if (this->lastUpdate_ == std::chrono::steady_clock::duration::zero()) {
    this->lastUpdate_ = _info.simTime;
    return;
  }
  
  // Calculate time delta
  std::chrono::duration<double> dt = _info.simTime - this->lastUpdate_;
  
  // Get current actor pose
  auto currentPose = gz::sim::worldPose(this->actorEntity_, _ecm);
  gz::math::Vector3d rpy = currentPose.Rot().Euler();
  
  gz::math::Pose3d newPose = currentPose;
  double distanceTraveled = 0.0;
  
  if (this->followMode_ == "path") {
    std::lock_guard<std::mutex> lock(this->mutex_);
    
    gz::math::Vector2d targetPos2d(this->targetPose_.X(), this->targetPose_.Y());
    gz::math::Vector2d currentPos2d(currentPose.Pos().X(), currentPose.Pos().Y());
    gz::math::Vector2d pos = targetPos2d - currentPos2d;
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
        pos.X() = 0;
        pos.Y() = 0;
      }
    }
    
    // Normalize the direction vector
    if (pos.Length() != 0) {
      pos = pos / pos.Length();
    }
    
    int rotSign = 1;
    double yawAngle = 0.0;
    
    // Calculate the angular displacement required
    if (pos.Length() != 0) {
      yawAngle = std::atan2(pos.Y(), pos.X()) + this->defaultRotation_ - rpy.Z();
      
      // Normalize angle to [-pi, pi]
      while (yawAngle > M_PI) yawAngle -= 2 * M_PI;
      while (yawAngle < -M_PI) yawAngle += 2 * M_PI;
    }
    
    if (yawAngle < 0)
      rotSign = -1;
    
    // Check if required angular displacement is greater than tolerance
    if (std::abs(yawAngle) > this->angTolerance_) {
      // Rotate towards target
      double newYaw = rpy.Z() + rotSign * this->angVelocity_ * dt.count();
      newPose.Rot() = gz::math::Quaterniond(this->defaultRotation_, 0, newYaw);
    } else {
      // Move towards the target position
      newPose.Pos().X() += pos.X() * this->linVelocity_ * dt.count();
      newPose.Pos().Y() += pos.Y() * this->linVelocity_ * dt.count();
      newPose.Rot() = gz::math::Quaterniond(this->defaultRotation_, 0, rpy.Z() + yawAngle);
      
      distanceTraveled = (pos * this->linVelocity_ * dt.count()).Length();
    }
    
  } else if (this->followMode_ == "velocity") {
    std::lock_guard<std::mutex> lock(this->mutex_);
    
    // Get velocity command from queue
    if (!this->cmdQueue_.empty()) {
      this->targetVel_.Pos().X() = this->cmdQueue_.front().X();
      this->targetVel_.Rot() = gz::math::Quaterniond(0, 0, this->cmdQueue_.front().Z());
      this->cmdQueue_.pop();
    }
    
    // Apply velocity
    double dx = this->targetVel_.Pos().X() *
                std::cos(currentPose.Rot().Euler().Z() - this->defaultRotation_) * dt.count();
    double dy = this->targetVel_.Pos().X() *
                std::sin(currentPose.Rot().Euler().Z() - this->defaultRotation_) * dt.count();
    
    newPose.Pos().X() += dx;
    newPose.Pos().Y() += dy;
    
    double newYaw = rpy.Z() + this->targetVel_.Rot().Euler().Z() * dt.count();
    newPose.Rot() = gz::math::Quaterniond(this->defaultRotation_, 0, newYaw);
    
    distanceTraveled = std::sqrt(dx * dx + dy * dy);
  }
  
  // Update actor pose
  _ecm.SetComponentData<gz::sim::components::Pose>(this->actorEntity_, newPose);
  
  // Update animation time using Actor helper class
  gz::sim::Actor actor(this->actorEntity_);
  auto currentAnimTime = actor.AnimationTime(_ecm);
  if (currentAnimTime) {
    std::chrono::duration<double> animTimeDelta(distanceTraveled * this->animationFactor_);
    auto newAnimTime = *currentAnimTime + std::chrono::duration_cast<std::chrono::steady_clock::duration>(animTimeDelta);
    actor.SetAnimationTime(_ecm, newAnimTime);
  }
  
  this->lastUpdate_ = _info.simTime;
}

/////////////////////////////////////////////////
void GazeboRosActorCommand::PostUpdate(
    const gz::sim::UpdateInfo &/*_info*/,
    const gz::sim::EntityComponentManager &/*_ecm*/) {
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
GZ_ADD_PLUGIN(
    gazebo_ros_actor_plugin::GazeboRosActorCommand,
    gz::sim::System,
    GazeboRosActorCommand::ISystemConfigure,
    GazeboRosActorCommand::ISystemPreUpdate,
    GazeboRosActorCommand::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gazebo_ros_actor_plugin::GazeboRosActorCommand,
    "gazebo_ros_actor_plugin::GazeboRosActorCommand")
