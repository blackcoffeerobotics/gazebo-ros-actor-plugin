#include <gazebo_ros_actor_plugin/gazebo_ros_path_follow_actor.h>

#include <functional>
#include <ignition/math.hh>
#include "gazebo/physics/physics.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(GazeboPathFollowRosActor)

#define WALKING_ANIMATION "walking"

/////////////////////////////////////////////////
GazeboPathFollowRosActor::GazeboPathFollowRosActor()
{
}

/////////////////////////////////////////////////
void GazeboPathFollowRosActor::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->active_topic_ = "active_state";
  if (_sdf->HasElement("active_state")){
    this->active_topic_ = _sdf->Get<std::string>("active_state");
  }

  this->ref_velocity_ = 0.5;
  if (_sdf->HasElement("ref_velocity")){
    this->ref_velocity_ = _sdf->Get<double>("ref_velocity");
  }

  this->traj_file_name_ = ros::package::getPath("gazebo_ros_actor_plugin")+"/traj/example.txt";
  if (_sdf->HasElement("traj_file_name")){
    this->traj_file_name_ = _sdf->Get<std::string>("traj_file_name");
  }

  this->animation_factor_ = 4.0;
  if (_sdf->HasElement("animation_factor")){
    this->animation_factor_ = _sdf->Get<double>("animation_factor");
  }

  this->oscillation_enable_ = false;
  if (_sdf->HasElement("oscillation_enable")){
    this->oscillation_enable_ = _sdf->Get<bool>("oscillation_enable");
  }

  this->ReadTrajectoryFile();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("actor", "A ROS node for Gazebo has not been initialized,"
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();

  this->Reset();

  this->ros_node_ = new ros::NodeHandle();

  // subscribe to the active topic
  ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::String>(this->active_topic_, 1,
                                                                             boost::bind(&GazeboPathFollowRosActor::OnActive, this, _1),
                                                                             ros::VoidPtr(), &queue_);
  this->state_sub_ = ros_node_->subscribe(so);

  // start custom queue for actor
  this->callbackQueueThread_ =
      boost::thread(boost::bind(&GazeboPathFollowRosActor::QueueThread, this));

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&GazeboPathFollowRosActor::OnUpdate, this, std::placeholders::_1)));
}

/////////////////////////////////////////////////
void GazeboPathFollowRosActor::Reset()
{
  this->last_update = 0;
  this->actor_state_ = "stop";

  auto skelAnims = this->actor->SkeletonAnimations();
  if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end())
  {
    gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
  }
  else
  {
    // Create custom trajectory
    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = WALKING_ANIMATION;
    this->trajectoryInfo->duration = 1.0;

    this->actor->SetCustomTrajectory(this->trajectoryInfo);
  }
}

void GazeboPathFollowRosActor::OnActive(const std_msgs::String::ConstPtr& msg)
{
  actor_state_ = msg->data;
}

/////////////////////////////////////////////////
void GazeboPathFollowRosActor::OnUpdate(const common::UpdateInfo &_info)
{
  ignition::math::Pose3d pose = this->actor->WorldPose();
  if(this->actor_state_ == "active")
  {
    ignition::math::Vector3d pos = trajectory_.front().Pos() - pose.Pos();
    ignition::math::Vector3d rpy = pose.Rot().Euler();
    if(pos.Length() < 0.05)
    {
      trajectory_.pop();
      pos = trajectory_.front().Pos() - pose.Pos();
    }
    pos = pos.Normalize();

    // Compute the yaw orientation
    ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + 1.5707 - rpy.Z();
    yaw.Normalize();

    double duration = (_info.simTime - this->start_time_).Double();
    double dt = (_info.simTime - this->last_update).Double();
    if(this->oscillation_enable_)
    {
      this->oscillation_factor_ = std::abs(std::sin(duration*4*M_PI/10.0));
      pose.Pos() += this->oscillation_factor_*pos*this->ref_velocity_*dt;
  //    ROS_INFO_STREAM(this->oscillation_factor_);
    }
    else
    {
      if (std::abs(yaw.Radian()) > IGN_DTOR(10))
      {
        pose.Pos() += pos * this->ref_velocity_ * dt; ///!
        pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+
            yaw.Radian()*0.001);
      }
      else
      {
        pose.Pos() += pos * this->ref_velocity_ * dt;
        pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+yaw.Radian());
      }
    }
  }
  else
  {
    // Add start time until start
    this->start_time_ = _info.simTime;
  }

  double distanceTraveled = (pose.Pos()-this->actor->WorldPose().Pos()).Length();

  this->actor->SetWorldPose(pose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime()+(distanceTraveled * this->animation_factor_));
  this->last_update = _info.simTime;
}

/////////////////////////////////////////////////
void GazeboPathFollowRosActor::QueueThread()
{
  static const double timeout = 0.01;

  while (this->ros_node_->ok())
    this->queue_.callAvailable(ros::WallDuration(timeout));
}

void GazeboPathFollowRosActor::ReadTrajectoryFile()
{
  std::ifstream trajectory_file;
  trajectory_file.open(traj_file_name_.c_str());
  if(!trajectory_file)
  {
    ROS_ERROR("Open trajectory file fail");
    return;
  }

  std::string line;
  while(!trajectory_file.eof())
  {
    std::getline(trajectory_file, line);
    std::istringstream in(line);
    ignition::math::Pose3d p;
    in >> p.Pos().X();
    in >> p.Pos().Y();
    p.Pos().Z() = 0.98;
    p.Rot().Euler().X() = 1.57;
    p.Rot().Euler().Z() = -1.57;
    trajectory_.push(p);
  }

  ROS_INFO_STREAM("Read trajectory from file");
  trajectory_file.close();
}
