#ifndef COMMAND_ACTOR_H
#define COMMAND_ACTOR_H

#include <string>
#include <queue>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{

/// \brief Gazebo plugin for commanding an actor to follow a path or velocity published by other ROS node.

class CommandActor : public ModelPlugin
{
  /// \brief Constructor
  public: CommandActor();

  /// \brief Destructor
  public: ~CommandActor();

  /// \brief Load the actor plugin.
  /// \param[in] _model Pointer to the parent model.
  /// \param[in] _sdf Pointer to the plugin's SDF elements.
  public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  // \brief Reset the plugin.
  public: virtual void Reset();

  /// \brief Callback function for receiving velocity commands from a publisher.
  /// \param[in] _model Pointer to the incoming velocity message.
  private: void VelCallback(const geometry_msgs::Twist::ConstPtr &msg);

  /// \brief Callback function for receiving path commands from a publisher.
  /// \param[in] _model Pointer to the incoming path message.
  private: void PathCallback(const nav_msgs::Path::ConstPtr &msg);

  /// \brief Function that is called every update cycle.
  /// \param[in] _info Timing information.
  private: void OnUpdate(const common::UpdateInfo &_info);

  /// \brief Custom callback queue thread for velocity commands.
  private: void VelQueueThread();

  /// \brief Custom callback queue thread for path commands.
  private: void PathQueueThread();

  /// \brief ROS node handle.
  private: ros::NodeHandle *ros_node_;

  /// \brief Subscribers for velocity and path commands.
  private: ros::Subscriber vel_sub_;
  private: ros::Subscriber path_sub_;

  /// \brief Custom callback queues for velocity and path commands.
  private: ros::CallbackQueue vel_queue_;
  private: ros::CallbackQueue path_queue_;

  /// \brief Custom callback queue threads for velocity and path commands.
  private: boost::thread velCallbackQueueThread_;
  private: boost::thread pathCallbackQueueThread_;

  /// \brief Topic names for velocity and path commands.
  private: std::string vel_topic_;
  private: std::string path_topic_;
  
  /// \brief Pointer to the parent actor.
  private: physics::ActorPtr actor;

  /// \brief Pointer to the world
  private: physics::WorldPtr world;

  /// \brief Pointer to the sdf element.
  private: sdf::ElementPtr sdf;

  /// \brief Animation factor for actor movement
  private: double animation_factor_;

  /// \brief List of connections
  private: std::vector<event::ConnectionPtr> connections;

  /// \brief Time of the last update.
  private: common::Time last_update;

  /// \brief Custom trajectory info.
  private: physics::TrajectoryInfoPtr trajectoryInfo;

  /// \brief Variable for control the non-constant velocity.
  private: bool oscillation_enable_;

  /// \brief Variable for control the non-constant velocity.
  private: double oscillation_factor_;

  /// \brief Flag to determine if the plugin will follow a path or velocity subscriber
  private: std::string follow_mode_;

  /// \brief Velocity of the robot guide // TODO: Discontinue the usage of guide terms
  private: ignition::math::Pose3d guide_vel_;

  /// \brief Linear and angular velocities of the actor when it follows a path
  private: double lin_velocity_;
  private: double ang_velocity_;

  /// \brief Factor to discretize the actor's yaw
  private: double spin_factor_;

  /// \brief Current target pose
  private: ignition::math::Vector3d target_pose;

  /// \brief List of target poses
  private: std::vector<ignition::math::Vector3d> target_poses;

  /// \brief Index of current target pose
  private: int idx;
  
  /// \brief Target pose Tolerance
  private: double lin_tolerance_;

  /// \brief Angular alignment Tolerance
  private: double ang_tolerance_;

  /// \brief Helper function to choose a new target pose
  private: void ChooseNewTarget();

  /// \brief Data structure for saving robot command.
  private: std::queue<ignition::math::Vector3d> cmd_queue_;

  /// \brief Flag for first run
  private: bool first_run_;

};
}

#endif // COMMAND_ACTOR_H
