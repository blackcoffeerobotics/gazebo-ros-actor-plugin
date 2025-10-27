#ifndef GAZEBO_ROS_ACTOR_PLUGIN_INCLUDE_GAZEBO_ROS_ACTOR_COMMAND
#define GAZEBO_ROS_ACTOR_PLUGIN_INCLUDE_GAZEBO_ROS_ACTOR_COMMAND

#include <string>
#include <queue>
#include <vector>
#include <memory>
#include <chrono>
#include <thread>
#include <mutex>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/EventManager.hh>
#include <ignition/gazebo/Actor.hh>
#include <ignition/gazebo/components/Actor.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/twist.pb.h>
#include <ignition/msgs/pose_v.pb.h>

#include <sdf/Element.hh>

namespace gazebo_ros_actor_plugin {

/// \brief Gazebo plugin for commanding an actor to follow
/// a path or velocity published by other ROS2 node.

class GazeboRosActorCommand :
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate,
    public ignition::gazebo::ISystemPostUpdate {
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
  void Configure(const ignition::gazebo::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 ignition::gazebo::EntityComponentManager &_ecm,
                 ignition::gazebo::EventManager &_eventMgr) override;

  /// \brief Called each simulation iteration
  /// \param[in] _info Simulation update info
  /// \param[in] _ecm Mutable reference to the EntityComponentManager
  void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                 ignition::gazebo::EntityComponentManager &_ecm) override;

  /// \brief Called after physics update
  /// \param[in] _info Simulation update info
  /// \param[in] _ecm Const reference to the EntityComponentManager
  void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                  const ignition::gazebo::EntityComponentManager &_ecm) override;

 private:
  /// \brief Callback function for receiving velocity commands from a publisher.
  /// \param[in] msg Pointer to the incoming velocity message.
  void VelCallback(const ignition::msgs::Twist &msg);

  /// \brief Callback function for receiving path commands from a publisher.
  /// \param[in] msg Pointer to the incoming path message (Pose_V = vector of poses).
  void PathCallback(const ignition::msgs::Pose_V &msg);

  /// \brief Helper function to choose a new target pose
  void ChooseNewTarget();

  /// \brief IGN transport node
  ignition::transport::Node node_;

  /// \brief Topic name for velocity commands
  std::string velTopic_;

  /// \brief Topic name for path commands
  std::string pathTopic_;

  /// \brief Entity ID of the actor
  ignition::gazebo::Entity actorEntity_;

  /// \brief Multiplier to base animation speed to adjust
  /// the speed of actor's animation and foot swinging
  double animationFactor_;

  /// \brief Time of the last update
  std::chrono::steady_clock::duration lastUpdate_;

  /// \brief Flag to determine if the plugin will follow a path or velocity
  std::string followMode_;

  /// \brief Target walking velocity for the actor
  ignition::math::Pose3d targetVel_;

  /// \brief Speed at which actor moves along path during path-following
  double linVelocity_;

  /// \brief Speed at which actor rotates to achieve desired orientation
  double angVelocity_;

  /// \brief Current target pose
  ignition::math::Vector3d targetPose_;

  /// \brief List of target poses
  std::vector<ignition::math::Vector3d> targetPoses_;

  /// \brief Index of current target pose
  int idx_;

  /// \brief Maximum allowed distance between actor and target pose
  double linTolerance_;

  /// \brief Maximum allowable difference in orientation
  double angTolerance_;

  /// \brief Default rotation for an actor
  double defaultRotation_;

  /// \brief Data structure for saving velocity command
  std::queue<ignition::math::Vector3d> cmdQueue_;

  /// \brief Data structure for saving path poses
  std::queue<std::vector<ignition::math::Vector3d>> pathQueue_;

  /// \brief Mutex for thread safety
  std::mutex mutex_;

  /// \brief Flag to track if path completion has been logged
  bool pathCompletedLogged_;
};

} // namespace gazebo_ros_actor_plugin

#endif // GAZEBO_ROS_ACTOR_PLUGIN_INCLUDE_GAZEBO_ROS_ACTOR_COMMAND
