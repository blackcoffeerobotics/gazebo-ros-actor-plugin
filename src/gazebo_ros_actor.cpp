#include <gazebo_ros_actor_plugin/gazebo_ros_actor.h>

#include <functional>

#include <ignition/math.hh>
#include "gazebo/physics/physics.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(GazeboRosActor)

#define WALKING_ANIMATION "walking"

/////////////////////////////////////////////////
GazeboRosActor::GazeboRosActor()
{
}

GazeboRosActor::~GazeboRosActor()
{
  this->vel_queue_.clear();
  this->vel_queue_.disable();
  this->velCallbackQueueThread_.join();
  this->ros_node_->shutdown();
  delete this->ros_node_;
}

/////////////////////////////////////////////////
void GazeboRosActor::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->vel_topic_ = "/cmd_vel_mux/input/navi";
  if (_sdf->HasElement("vel_topic"))
  {
    this->vel_topic_ = _sdf->Get<std::string>("vel_topic");
  }

  this->velocity_ = 0.4;
  if (_sdf->HasElement("velocity"))
  {
    this->velocity_ = _sdf->Get<double>("velocity");
  }

  this->trajectory_type_ = 0;
  if (_sdf->HasElement("trajectory_type"))
  {
    this->trajectory_type_ = _sdf->Get<int>("trajectory_type");
  }

  this->animation_factor_ = 4.0;
  if (_sdf->HasElement("animation_factor"))
  {
    this->animation_factor_ = _sdf->Get<double>("animation_factor");
  }

  this->oscillation_enable_ = false;
  if (_sdf->HasElement("oscillation_enable"))
  {
    this->oscillation_enable_ = _sdf->Get<bool>("oscillation_enable");
  }

  // this->ReadTrajectoryFile();

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

  first_run_ = true;

  this->ros_node_ = new ros::NodeHandle();

  // subscribe to the speed of the guide
  ros::SubscribeOptions vel_so = ros::SubscribeOptions::create<geometry_msgs::Twist>(vel_topic_, 1,
                                                                                     boost::bind(&GazeboRosActor::VelCallback, this, _1),
                                                                                     ros::VoidPtr(), &vel_queue_);
  this->vel_sub_ = ros_node_->subscribe(vel_so);

  this->velCallbackQueueThread_ =
      boost::thread(boost::bind(&GazeboRosActor::VelQueueThread, this));

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboRosActor::OnUpdate, this, std::placeholders::_1)));
}

/////////////////////////////////////////////////
void GazeboRosActor::Reset()
{
  this->last_update = 0;

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

void GazeboRosActor::VelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  ignition::math::Vector3d cmd;
  if (first_run_)
  {
    /// Insert initial commands
    for (int i = 0; i < 1000*2/0.4; i++)
    {
      cmd.X() = 0.4;
      cmd.Z() = 0.0;
      cmd_queue_.push(cmd);
    }
    first_run_ = false;
  }

  cmd.X() = msg->linear.x;
  cmd.Z() = msg->angular.z;
  cmd_queue_.push(cmd);
}

/////////////////////////////////////////////////
void GazeboRosActor::OnUpdate(const common::UpdateInfo &_info)
{
  ignition::math::Pose3d pose = this->actor->WorldPose();
  double dt = (_info.simTime - this->last_update).Double();
  ignition::math::Vector3d rpy = pose.Rot().Euler();

  if (!cmd_queue_.empty())
  {
    this->guide_vel_.Pos().X() = cmd_queue_.front().X();
    this->guide_vel_.Rot() = ignition::math::Quaterniond(0, 0, cmd_queue_.front().Z());
    cmd_queue_.pop();
  }

  pose.Pos().X() += this->guide_vel_.Pos().X()*cos(pose.Rot().Euler().Z()-1.57)*dt;
  pose.Pos().Y() -= this->guide_vel_.Pos().X()*sin(pose.Rot().Euler().Z()-1.57)*dt;

  pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()-this->guide_vel_.Rot().Euler().Z()*dt);

  double distanceTraveled = (pose.Pos() - this->actor->WorldPose().Pos()).Length();

  this->actor->SetWorldPose(pose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime() + (distanceTraveled * this->animation_factor_));
  this->last_update = _info.simTime;
}

void GazeboRosActor::VelQueueThread()
{
  static const double timeout = 0.01;

  while (this->ros_node_->ok())
    this->vel_queue_.callAvailable(ros::WallDuration(timeout));
}

void GazeboRosActor::ReadTrajectoryFile()
{
  std::ifstream trajectory_file;
  std::string file_path;
  switch (trajectory_type_)
  {
  case 0:
    file_path = ros::package::getPath("turtlebot_guide_integration") + "/trajectories/scene_0.txt";
    break;
  case 1:
    file_path = ros::package::getPath("turtlebot_guide_integration") + "/trajectories/scene_1.txt";
    break;

  default:
    break;
  }

  trajectory_file.open(file_path.c_str());
  if (!trajectory_file)
  {
    ROS_ERROR("Open trajectory file fail");
    return;
  }

  std::string line;
  while (!trajectory_file.eof())
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
