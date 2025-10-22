#ifndef GAZEBO_ROS_ACTOR_PLUGIN_INCLUDE_GAZEBO_ROS_ACTOR_COMMAND
#define GAZEBO_ROS_ACTOR_PLUGIN_INCLUDE_GAZEBO_ROS_ACTOR_COMMAND

#include <string>
#include <queue>
#include <vector>
#include <memory>
#include <chrono>
#include <thread>
#include <mutex>

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Actor.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>

#include <sdf/Element.hh>

namespace gazebo_ros_actor_plugin {

/// \brief Gazebo plugin for commanding an actor to follow
/// a path or velocity published by other ROS2 node.

class GazeboRosActorCommand :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate {
 public:
  /// \brief Constructor
  GazeboRosActorCommand();

  /// \brief Destructor
  ~GazeboRosActorCommand() override;

  /// \brief Configure the system
  /// \param[in] _entity The entity this plugin is attached to
  /// \param[in] _sdf The SDF Element associated with this system plugin
  /// \param[in] _ecm The EntityComponentManager of the given simulation instance
  /// \param[in] _eventMgr The EventManager of the given simulation instance
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  /// \brief Called each simulation iteration
  /// \param[in] _info Simulation update info
  /// \param[in] _ecm Mutable reference to the EntityComponentManager
  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

  /// \brief Called after physics update
  /// \param[in] _info Simulation update info
  /// \param[in] _ecm Const reference to the EntityComponentManager
  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) override;

 private:
  /// \brief Callback function for receiving velocity commands from a publisher.
  /// \param[in] msg Pointer to the incoming velocity message.
  void VelCallback(const gz::msgs::Twist &msg);

  /// \brief Helper function to choose a new target pose
  void ChooseNewTarget();

  /// \brief GZ transport node
  gz::transport::Node node_;

  /// \brief Topic name for velocity commands
  std::string velTopic_;

  /// \brief Entity ID of the actor
  gz::sim::Entity actorEntity_;

  /// \brief Multiplier to base animation speed to adjust
  /// the speed of actor's animation and foot swinging
  double animationFactor_;

  /// \brief Time of the last update
  std::chrono::steady_clock::duration lastUpdate_;

  /// \brief Flag to determine if the plugin will follow a path or velocity
  std::string followMode_;

  /// \brief Target walking velocity for the actor
  gz::math::Pose3d targetVel_;

  /// \brief Speed at which actor moves along path during path-following
  double linVelocity_;

  /// \brief Speed at which actor rotates to achieve desired orientation
  double angVelocity_;

  /// \brief Current target pose
  gz::math::Vector3d targetPose_;

  /// \brief List of target poses
  std::vector<gz::math::Vector3d> targetPoses_;

  /// \brief Index of current target pose
  int idx_;

  /// \brief Maximum allowed distance between actor and target pose
  double linTolerance_;

  /// \brief Maximum allowable difference in orientation
  double angTolerance_;

  /// \brief Default rotation for an actor
  double defaultRotation_;

  /// \brief Data structure for saving velocity command
  std::queue<gz::math::Vector3d> cmdQueue_;

  /// \brief Mutex for thread safety
  std::mutex mutex_;
};

} // namespace gazebo_ros_actor_plugin

#endif // GAZEBO_ROS_ACTOR_PLUGIN_INCLUDE_GAZEBO_ROS_ACTOR_COMMAND
