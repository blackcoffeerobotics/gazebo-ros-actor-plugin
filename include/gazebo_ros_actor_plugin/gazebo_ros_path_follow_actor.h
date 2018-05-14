#ifndef GAZEBO_ROS_PATH_FOLLOW_ACTOR_H
#define GAZEBO_ROS_PATH_FOLLOW_ACTOR_H

#include <string>
#include <fstream>
#include <queue>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{

class GazeboPathFollowRosActor : public ModelPlugin
{
  /// \brief Constructor
  public: GazeboPathFollowRosActor();

  public: ~GazeboPathFollowRosActor();

    /// \brief Load the actor plugin.
    /// \param[in] _model Pointer to the parent model.
    /// \param[in] _sdf Pointer to the plugin's SDF elements.
  public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// Documentation Inherited.
  public: virtual void Reset();

    /// \brief Function that is called every update cycle.
    /// \param[in] _info Timing information
  private: void OnUpdate(const common::UpdateInfo &_info);

  /// \brief callback for updating the activation of the actor
  private: void OnActive(const std_msgs::String::ConstPtr& msg);

  private: void QueueThread();

  private: void ReadTrajectoryFile();

  private: ros::NodeHandle *ros_node_;

  private: ros::Subscriber state_sub_;

  /// \brief Custom Callback Queue
  private: ros::CallbackQueue queue_;

  /// \brief Custom Callback Queue thread
  private: boost::thread callbackQueueThread_;

  /// \brief Pointer to the parent actor.
  private: physics::ActorPtr actor;

  /// \brief Pointer to the world, for convenience.
  private: physics::WorldPtr world;

  /// \brief Pointer to the sdf element.
  private: sdf::ElementPtr sdf;

   /// \brief Reference velocity of the actor
  private: ignition::math::Vector3d ref_velocity_;

  /// \brief Time scaling factor. Used to coordinate translational motion
  /// with the actor's walking animation.
  private: double animation_factor_;

  /// \brief List of connections
  private: std::vector<event::ConnectionPtr> connections;

  /// \brief Time of the last update.
  private: common::Time last_update;

  /// \brief Time of start.
  private: common::Time start_time_;

  /// \brief Custom trajectory info.
  private: physics::TrajectoryInfoPtr trajectoryInfo;

  /// \brief Variable for control the non-constant velocity.
  private: bool oscillation_enable_;

  /// \brief To make the actor to have non-constant velocity.
  private: double oscillation_factor_;

  private: std::string active_topic_;

  private: std::string actor_state_;

  private: std::string traj_file_name_;

  /// \brief Data structure for saving the trajectory read from file
  private: std::queue<ignition::math::Pose3d> trajectory_;

};
}

#endif // GAZEBO_ROS_PATH_FOLLOW_ACTOR_H
