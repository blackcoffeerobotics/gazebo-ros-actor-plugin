#ifndef GAZEBO_ROS_ACTOR_H
#define GAZEBO_ROS_ACTOR_H

#include <string>
#include <fstream>
#include <queue>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{

class GazeboRosActor : public ModelPlugin
{
  /// \brief Constructor
  public: GazeboRosActor();

  public: ~GazeboRosActor();

    /// \brief Load the actor plugin.
    /// \param[in] _model Pointer to the parent model.
    /// \param[in] _sdf Pointer to the plugin's SDF elements.
  public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Inherited.
  public: virtual void Reset();

  private: void OnActive(const std_msgs::String::ConstPtr &msg);
  
  private: void VelCallback(const geometry_msgs::Twist::ConstPtr &msg);

    /// \brief Function that is called every update cycle.
    /// \param[in] _info Timing information
  private: void OnUpdate(const common::UpdateInfo &_info);

  private: void QueueThread();

  private: void VelQueueThread();

  private: void ReadTrajectoryFile();

  private: ros::NodeHandle *ros_node_;

  private: ros::Subscriber sub_;
  
  private: ros::Subscriber vel_sub_;

  /// \brief Custom Callback Queue for activating actor
  private: ros::CallbackQueue queue_;

  /// \brief Custom Callback Queue for guide vel
  private: ros::CallbackQueue vel_queue_;

  /// \brief Custom Callback Queue thread
  private: boost::thread callbackQueueThread_;

  private: boost::thread velCallbackQueueThread_;

  private: std::string topic_;

  private: std::string vel_topic_;

  /// \brief Pointer to the parent actor.
  private: physics::ActorPtr actor;

  /// \brief Pointer to the world, for convenience.
  private: physics::WorldPtr world;

  /// \brief Pointer to the sdf element.
  private: sdf::ElementPtr sdf;

   /// \brief Velocity of the actor
  private: ignition::math::Vector3d velocity_;

  /// \brief Velocity of the robot guide
  private: ignition::math::Pose3d guide_vel_;

  private: double animation_factor_;

  /// \brief List of connections
  private: std::vector<event::ConnectionPtr> connections;

  /// \brief Time of the last update.
  private: common::Time last_update;

  /// \brief Time of the trajectory update.
  private: common::Time start_time_;

  /// \brief Custom trajectory info.
  private: physics::TrajectoryInfoPtr trajectoryInfo;

  /// \brief Variable for control the non-constant velocity.
  private: bool oscillation_enable_;

  /// \brief Variable for control the non-constant velocity.
  private: double oscillation_factor_;

  /// \brief Queue data structure for storing the trajectory data
  private: std::queue<ignition::math::Pose3d> trajectory_;

  private: int trajectory_type_;

  private: std::string nav_state_;
};
}
#endif
