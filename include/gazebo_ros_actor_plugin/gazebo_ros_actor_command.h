#pragma once

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/Angle.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Actor.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/transport/Node.hh>

#include <sdf/Element.hh>

namespace gazebo_ros_actor_plugin {

class GazeboRosActorCommand :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
 public:
   GazeboRosActorCommand();
   void Configure(const gz::sim::Entity &_entity,
                  const std::shared_ptr<const sdf::Element> &_sdf,
                  gz::sim::EntityComponentManager &_ecm,
                  gz::sim::EventManager &_eventMgr) override;
   void PreUpdate(const gz::sim::UpdateInfo &_info,
                  gz::sim::EntityComponentManager &_ecm) override;

 private:
   void VelCallback(const gz::msgs::Twist &msg);
   void PathCallback(const gz::msgs::Pose_V &msg);
   void ChooseNewTarget();

   gz::transport::Node node_;
   std::string velTopic_;
   std::string pathTopic_;
   gz::sim::Entity actorEntity_;
   double animationFactor_;
   std::chrono::steady_clock::duration lastUpdate_;
   std::string followMode_;
   gz::math::Pose3d targetVel_;
   double linVelocity_;
   double angVelocity_;
   gz::math::Vector3d targetPose_;
   std::vector<gz::math::Vector3d> targetPoses_;
   int idx_;
   double linTolerance_;
   double angTolerance_;
   double defaultRotation_;
   double streetHeight_;
   std::queue<gz::math::Vector3d> cmdQueue_;
   std::queue<std::vector<gz::math::Vector3d>> pathQueue_;
   std::mutex mutex_;
   bool pathCompletedLogged_;
};

} // namespace gazebo_ros_actor_plugin

