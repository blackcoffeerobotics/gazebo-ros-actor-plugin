#include <gazebo_ros_actor_plugin/gazebo_ros_actor_command.h>

using namespace gazebo_ros_actor_plugin;

GazeboRosActorCommand::GazeboRosActorCommand()
  : actorEntity_(gz::sim::kNullEntity),
    animationFactor_(4.0),
    lastUpdate_(std::chrono::steady_clock::duration::zero()),
    followMode_("velocity"),
    targetVel_(gz::math::Pose3d::Zero),
    linVelocity_(1.0),
    angVelocity_(GZ_DTOR(10)),
    idx_(0),
    linTolerance_(0.1),
    angTolerance_(GZ_DTOR(5)),
    defaultRotation_(M_PI/2),
    pathCompletedLogged_(false) {
}

void GazeboRosActorCommand::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &/*_eventMgr*/) {

  this->actorEntity_ = _entity;

  auto actorComp = _ecm.Component<gz::sim::components::Actor>(this->actorEntity_);
  if (!actorComp)
  {
    gzerr << "Entity [" << _entity << "] is not an actor." << std::endl;
    return;
  }

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

  std::string animationName;

  // If animation not provided, use first one from SDF
  if (!_sdf->HasElement("animation"))
  {
    if (actorComp->Data().AnimationCount() < 1)
    {
      gzerr << "Actor SDF doesn't have any animations." << std::endl;
      return;
    }

    animationName = actorComp->Data().AnimationByIndex(0)->Name();
  }
  else
  {
    animationName = _sdf->Get<std::string>("animation");
  }

  if (animationName.empty())
  {
    gzerr << "Can't find actor's animation name." << std::endl;
    return;
  }

  auto animationNameComp = _ecm.Component<gz::sim::components::AnimationName>(_entity);
  if (nullptr == animationNameComp)
  {
    _ecm.CreateComponent(_entity, gz::sim::components::AnimationName(animationName));
  }
  else
  {
    *animationNameComp = gz::sim::components::AnimationName(animationName);
  }
  // Mark as a one-time-change so that the change is propagated to the GUI
  _ecm.SetChanged(_entity,
    gz::sim::components::AnimationName::typeId, gz::sim::ComponentState::OneTimeChange);

  // Set custom animation time from this plugin
  auto animTimeComp = _ecm.Component<gz::sim::components::AnimationTime>(_entity);
  if (nullptr == animTimeComp)
  {
    _ecm.CreateComponent(_entity, gz::sim::components::AnimationTime());
  }

  gz::math::Pose3d initialPose;
  auto poseComp = _ecm.Component<gz::sim::components::Pose>(_entity);
  if (nullptr == poseComp)
  {
    _ecm.CreateComponent(_entity, gz::sim::components::Pose(
      gz::math::Pose3d::Zero));
  }
  else
  {
    initialPose = poseComp->Data();

    // We'll be setting the actor's X/Y pose with respect to the world. So we
    // zero the current values.
    auto newPose = initialPose;
    newPose.Pos().X(0);
    newPose.Pos().Y(0);
    *poseComp = gz::sim::components::Pose(newPose);
  }

  // Having a trajectory pose prevents the actor from moving with the
  // SDF script
  auto trajPoseComp = _ecm.Component<gz::sim::components::TrajectoryPose>(_entity);
  if (nullptr == trajPoseComp)
  {
    // Leave Z to the pose component, control only 2D with Trajectory
    initialPose.Pos().Z(0);
    _ecm.CreateComponent(_entity, gz::sim::components::TrajectoryPose(initialPose));
  }

  if (!this->node_.Subscribe(this->velTopic_, &GazeboRosActorCommand::VelCallback, this)) {
    gzerr << "Failed to subscribe to velocity topic: " << this->velTopic_ << std::endl;
  }

  if (!this->node_.Subscribe(this->pathTopic_, &GazeboRosActorCommand::PathCallback, this)) {
    gzerr << "Failed to subscribe to path topic: " << this->pathTopic_ << std::endl;
  }

  this->lastUpdate_ = std::chrono::steady_clock::duration::zero();
}

void GazeboRosActorCommand::VelCallback(const gz::msgs::Twist &msg) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  gz::math::Vector3d velCmd;
  velCmd.X() = msg.linear().x();
  velCmd.Z() = msg.angular().z();
  this->cmdQueue_.push(velCmd);
}

void GazeboRosActorCommand::PathCallback(const gz::msgs::Pose_V &msg) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  std::vector<gz::math::Vector3d> poses;

  for (int i = 0; i < msg.pose_size(); ++i) {
    const auto& pose = msg.pose(i);
    double x = pose.position().x();
    double y = pose.position().y();

    gz::math::Quaterniond quat(
      pose.orientation().w(),
      pose.orientation().x(),
      pose.orientation().y(),
      pose.orientation().z()
    );
    double yaw = quat.Euler().Z();

    poses.push_back(gz::math::Vector3d(x, y, yaw));
  }

  if (!poses.empty()) {
    this->pathQueue_.push(poses);
    gzmsg << "New path received with " << poses.size() << " waypoints" << std::endl;
  } else {
    gzwarn << "Received empty path" << std::endl;
  }
}

void GazeboRosActorCommand::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) {

  GZ_PROFILE("GazeboRosActorCommand::PreUpdate");

  std::chrono::duration<double> dt = _info.simTime - this->lastUpdate_;
  this->lastUpdate_ = _info.simTime;

  auto trajPoseComp = _ecm.Component<gz::sim::components::TrajectoryPose>(this->actorEntity_);
  auto actorPose = trajPoseComp->Data();
  auto currentPose = actorPose;

  gz::math::Vector3d rpy = currentPose.Rot().Euler();

  gz::math::Pose3d newPose = currentPose;
  double distanceTraveled = 0.0;

  if (this->followMode_ == "path") {
    std::lock_guard<std::mutex> lock(this->mutex_);

    if (!this->pathQueue_.empty()) {
      this->targetPoses_ = this->pathQueue_.front();
      this->pathQueue_.pop();

      this->idx_ = 0;
      if (!this->targetPoses_.empty()) {
        this->targetPose_ = this->targetPoses_.at(this->idx_);
        this->pathCompletedLogged_ = false;
        gzmsg << "New path loaded with " << this->targetPoses_.size()
          << " waypoints" << std::endl;
      }
    }

    if (this->targetPoses_.empty() || this->idx_ >= static_cast<int>(this->targetPoses_.size())) {
      this->lastUpdate_ = _info.simTime;
      return;
    }

    gz::math::Vector2d targetPos2d(this->targetPose_.X(), this->targetPose_.Y());
    gz::math::Vector2d currentPos2d(currentPose.Pos().X(), currentPose.Pos().Y());
    gz::math::Vector2d pos = targetPos2d - currentPos2d;
    double distance = pos.Length();

    if (distance < this->linTolerance_) {
      if (this->idx_ < static_cast<int>(this->targetPoses_.size()) - 1) {
        this->ChooseNewTarget();
        pos.X() = this->targetPose_.X() - currentPose.Pos().X();
        pos.Y() = this->targetPose_.Y() - currentPose.Pos().Y();
      } else {
        if (!this->pathCompletedLogged_) {
          gzmsg << "Path completed - all waypoints reached" << std::endl;
          this->pathCompletedLogged_ = true;
        }
        pos.X() = 0;
        pos.Y() = 0;
      }
    }

    if (pos.Length() != 0) {
      pos = pos / pos.Length();
    }

    double targetYaw = std::atan2(pos.Y(), pos.X());

    newPose.Rot() = gz::math::Quaterniond(0, 0, targetYaw);

    if (pos.Length() != 0) {
      newPose.Pos().X() += pos.X() * this->linVelocity_ * dt.count();
      newPose.Pos().Y() += pos.Y() * this->linVelocity_ * dt.count();
      distanceTraveled = (pos * this->linVelocity_ * dt.count()).Length();
    }

  } else if (this->followMode_ == "velocity") {
    std::lock_guard<std::mutex> lock(this->mutex_);

    if (!this->cmdQueue_.empty()) {
      gz::math::Vector3d vel = this->cmdQueue_.front();
      this->cmdQueue_.pop();

      this->targetVel_.Pos().X() = vel.X();
      this->targetVel_.Rot() = gz::math::Quaterniond(0, 0, vel.Z());
    }

    if (std::abs(this->targetVel_.Pos().X()) > 0.001 ||
        std::abs(this->targetVel_.Rot().Euler().Z()) > 0.001) {
      double dx = this->targetVel_.Pos().X() *
        std::cos(currentPose.Rot().Euler().Z()) * dt.count();
      double dy = this->targetVel_.Pos().X() *
        std::sin(currentPose.Rot().Euler().Z()) * dt.count();

      newPose.Pos().X() += dx;
      newPose.Pos().Y() += dy;

      double newYaw = rpy.Z() + this->targetVel_.Rot().Euler().Z() * dt.count();
      newPose.Rot() = gz::math::Quaterniond(0, 0, newYaw);

      distanceTraveled = std::sqrt(dx * dx + dy * dy);
    } else {
      this->targetVel_ = gz::math::Pose3d::Zero;
    }
  }

  *trajPoseComp = gz::sim::components::TrajectoryPose(newPose);

  _ecm.SetChanged(
    this->actorEntity_,
    gz::sim::components::TrajectoryPose::typeId,
    gz::sim::ComponentState::OneTimeChange);

  // Update actor bone trajectories based on animation time
  auto animTimeComp = _ecm.Component<gz::sim::components::AnimationTime>(this->actorEntity_);

  if (distanceTraveled > 0.0001) {
    auto animTime = animTimeComp->Data() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(distanceTraveled * this->animationFactor_)
      );

    *animTimeComp = gz::sim::components::AnimationTime(animTime);

    _ecm.SetChanged(
      this->actorEntity_,
      gz::sim::components::AnimationTime::typeId,
      gz::sim::ComponentState::OneTimeChange);
  }
}

void GazeboRosActorCommand::ChooseNewTarget() {
  this->idx_++;

  if (this->idx_ < static_cast<int>(this->targetPoses_.size())) {
    this->targetPose_ = this->targetPoses_.at(this->idx_);
  }
}

GZ_ADD_PLUGIN(
  gazebo_ros_actor_plugin::GazeboRosActorCommand,
  gz::sim::System,
  GazeboRosActorCommand::ISystemConfigure,
  GazeboRosActorCommand::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  gazebo_ros_actor_plugin::GazeboRosActorCommand,
  "gazebo_ros_actor_plugin::GazeboRosActorCommand")
