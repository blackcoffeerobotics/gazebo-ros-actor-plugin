#include <gazebo_ros_actor_plugin/gazebo_ros_robot_follow_actor.h>

#include <functional>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <ignition/math.hh>
#include "gazebo/physics/physics.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(GazeboRosRobotFollowActor)

#define WALKING_ANIMATION "walking"

/////////////////////////////////////////////////
GazeboRosRobotFollowActor::GazeboRosRobotFollowActor()
{
}

GazeboRosRobotFollowActor::~GazeboRosRobotFollowActor()
{
  this->vel_queue_.clear();
  this->vel_queue_.disable();
  this->velCallbackQueueThread_.join();

  // Added for path
  this->path_queue_.clear();
  this->path_queue_.disable();
  this->pathCallbackQueueThread_.join();

  this->ros_node_->shutdown();
  delete this->ros_node_; 
}

/////////////////////////////////////////////////
void GazeboRosRobotFollowActor::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{

  gzdbg << "Load function debug message"<< std::endl;

  this->vel_topic_ = "/cmd_vel_mux/input/navi";
  if (_sdf->HasElement("vel_topic"))
  {
    this->vel_topic_ = _sdf->Get<std::string>("vel_topic");
  }

  // Added for path
  this->path_topic_ = "/cmd_path";
  // if (_sdf->HasElement("path_topic"))
  // {
  //   this->path_topic_ = _sdf->Get<std::string>("path_topic");
  // }

  this->animation_factor_ = 4.0;
  if (_sdf->HasElement("animation_factor"))
  {
    this->animation_factor_ = _sdf->Get<double>("animation_factor");
  }

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

  // Set index to zero
  this->idx = 0;
  this->tolerance = 0.1;

  this->Reset();

  this->ros_node_ = new ros::NodeHandle();

  // subscribe to the speed of the guide
  ros::SubscribeOptions vel_so = ros::SubscribeOptions::create<geometry_msgs::Twist>(vel_topic_, 1000,
                                                                                     boost::bind(&GazeboRosRobotFollowActor::VelCallback, this, _1),
                                                                                     ros::VoidPtr(), &vel_queue_);
  this->vel_sub_ = ros_node_->subscribe(vel_so);

  this->velCallbackQueueThread_ =
      boost::thread(boost::bind(&GazeboRosRobotFollowActor::VelQueueThread, this));

  // subscribe to the pose of the guide
  ros::SubscribeOptions path_so = ros::SubscribeOptions::create<nav_msgs::Path>(path_topic_, 1000,
                                                                                     boost::bind(&GazeboRosRobotFollowActor::PathCallback, this, _1),
                                                                                     ros::VoidPtr(), &path_queue_);
  this->path_sub_ = ros_node_->subscribe(path_so);

  this->pathCallbackQueueThread_ =
      boost::thread(boost::bind(&GazeboRosRobotFollowActor::PathQueueThread, this));

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboRosRobotFollowActor::OnUpdate, this, std::placeholders::_1)));
}

/////////////////////////////////////////////////
void GazeboRosRobotFollowActor::Reset()
{

  this->last_update = 0;
  this->lin_velocity = 1;
  this->spin_factor = 0.01;
  
  gzdbg << "Improve the initialization of target poses vector to current pose. Format: [x,y,yaw]"<< std::endl;
  this->target_poses.push_back(ignition::math::Vector3d(2.0, 0.0, 0.0));

  if (this->target_poses.empty()) {
      gzdbg << "Target poses vector is empty" << std::endl;
  }

  if (this->idx >= this->target_poses.size()) {
      gzdbg << "Reached end of target poses vector" << std::endl;
  }
 
  this->target_pose = this->target_poses.at(this->idx);
  // this->target_pose = this->target_poses[this->idx];

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

  // TODO: Make changes as per the second subscriber

}

void GazeboRosRobotFollowActor::VelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  ignition::math::Vector3d vel_cmd;
  vel_cmd.X() = msg->linear.x;
  vel_cmd.Z() = msg->angular.z;
  this->cmd_queue_.push(vel_cmd);
}

void GazeboRosRobotFollowActor::PathCallback(const nav_msgs::Path::ConstPtr &msg)
{
  // Extract the poses from the Path message
  const std::vector<geometry_msgs::PoseStamped>& poses = msg->poses;

  // gzdbg << "PathCallback function!" << std::endl;

  // Extract the x, y, and yaw from each pose and store it as a target
  for (size_t i = 0; i < poses.size(); ++i)
  {
    const geometry_msgs::Pose& pose = poses[i].pose;
    const double x = pose.position.x;
    const double y = pose.position.y;

    // Convert quaternion to Euler angles
    tf2::Quaternion quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf2::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    // gzdbg << "x = " << x << ", y = " << y << ", yaw = " << yaw << std::endl;
    this->target_poses.push_back(ignition::math::Vector3d(x, y, yaw));
  }
}

/////////////////////////////////////////////////
void GazeboRosRobotFollowActor::OnUpdate(const common::UpdateInfo &_info)
{
  // Time delta
  double dt = (_info.simTime - this->last_update).Double();
  
  ignition::math::Pose3d pose = this->actor->WorldPose();
  
  ignition::math::Vector3d rpy = pose.Rot().Euler();

  // IF cmd_vel
  // if (!this->cmd_queue_.empty())
  // {
  //   this->guide_vel_.Pos().X() = this->cmd_queue_.front().X();
  //   this->guide_vel_.Rot() = ignition::math::Quaterniond(0, 0, this->cmd_queue_.front().Z());
  //   this->cmd_queue_.pop();
  // }

  // pose.Pos().X() += this->guide_vel_.Pos().X()*sin(pose.Rot().Euler().Z())*dt;
  // pose.Pos().Y() -= this->guide_vel_.Pos().X()*cos(pose.Rot().Euler().Z())*dt;

  pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+this->guide_vel_.Rot().Euler().Z()*dt);

  /////////////////////////////////////////////////////////////////////////////////////////

  // // IF cmd_path

  ignition::math::Vector2d target_pos_2d(this->target_pose.X(), this->target_pose.Y());
  ignition::math::Vector2d current_pos_2d(pose.Pos().X(), pose.Pos().Y());
  ignition::math::Vector2d pos = target_pos_2d - current_pos_2d;
  double distance = pos.Length();

  // Choose a new target position if the actor has reached its current target.
  if (distance < this->tolerance)
  {
    // If there are more targets, change target
    if (this->idx < this->target_poses.size() - 1){
    this->ChooseNewTarget();
    gzdbg << "Pursuing next target!" << std::endl; 
    pos.X() = this->target_pose.X() - pose.Pos().X();
    pos.Y() = this->target_pose.Y() - pose.Pos().Y();    
    }
    // If all targets have been accomplished, move no further
    else{
      gzdbg << "Last target reached!" << std::endl; 
      pos.X() = 0;
      pos.Y() = 0;
    }
  }

  // Normalize the direction vector
  // pos = pos.Normalize(); // Doesn't work
  if (pos.Length() != 0){
    pos = pos/pos.Length();  
  }

  // Compute the yaw orientation
  ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + 1.5707 - rpy.Z();
  yaw.Normalize();

  // Rotate in place, instead of jumping [If yaw>10 deg]
  if (std::abs(yaw.Radian()) > IGN_DTOR(10))
  {
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+
        yaw.Radian()*this->spin_factor);
  }
  else
  {
    // pose.Pos() += pos * this->lin_velocity * dt;
    pose.Pos().X() += pos.X() * this->lin_velocity * dt;
    pose.Pos().Y() += pos.Y() * this->lin_velocity * dt;

    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+yaw.Radian());
  }

  /////////////////////////////////////////////////////////////////////////////////////////

  // After IF-ELSE: Standard post processing 
  
  // Distance traveled is used to coordinate motion with the walking animation
  double distanceTraveled = (pose.Pos() - this->actor->WorldPose().Pos()).Length();

  this->actor->SetWorldPose(pose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime() + (distanceTraveled * this->animation_factor_));
  this->last_update = _info.simTime;
}

void GazeboRosRobotFollowActor::ChooseNewTarget()
{
  this->idx++;

  // Set next target
  this->target_pose = this->target_poses.at(this->idx);  
}

void GazeboRosRobotFollowActor::VelQueueThread()
{
  static const double timeout = 0.01;

  while (this->ros_node_->ok())
    this->vel_queue_.callAvailable(ros::WallDuration(timeout));
}

void GazeboRosRobotFollowActor::PathQueueThread()
{
  static const double timeout = 0.01;

  while (this->ros_node_->ok())
    this->path_queue_.callAvailable(ros::WallDuration(timeout));
}