// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/pose_v.pb.h>

namespace gazebo_ros_actor_plugin
{

class PathBridgeNode : public rclcpp::Node
{
public:
  PathBridgeNode()
  : Node("path_bridge_node")
  {
    // Declare parameters
    this->declare_parameter<std::string>("ros_path_topic", "/cmd_path");
    this->declare_parameter<std::string>("gz_path_topic", "/cmd_path");
    
    // Get parameters
    std::string ros_path_topic = this->get_parameter("ros_path_topic").as_string();
    std::string gz_path_topic = this->get_parameter("gz_path_topic").as_string();
    
    // Create ROS2 subscriber
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      ros_path_topic,
      10,
      std::bind(&PathBridgeNode::pathCallback, this, std::placeholders::_1)
    );
    
    // Create Gazebo publisher
    gz_pub_ = gz_node_.Advertise<ignition::msgs::Pose_V>(gz_path_topic);
    
    if (!gz_pub_)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to create Gazebo publisher on topic: %s", gz_path_topic.c_str());
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Path bridge node started");
      RCLCPP_INFO(this->get_logger(), "  ROS2 topic: %s", ros_path_topic.c_str());
      RCLCPP_INFO(this->get_logger(), "  Gazebo topic: %s", gz_path_topic.c_str());
    }
  }

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Received empty path, ignoring");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Received ROS2 path with %zu waypoints", msg->poses.size());
    
    // Convert ROS2 Path to Gazebo Pose_V
    ignition::msgs::Pose_V gz_path;
    
    for (const auto& pose_stamped : msg->poses)
    {
      auto* pose = gz_path.add_pose();
      
      // Set position
      pose->mutable_position()->set_x(pose_stamped.pose.position.x);
      pose->mutable_position()->set_y(pose_stamped.pose.position.y);
      pose->mutable_position()->set_z(pose_stamped.pose.position.z);
      
      // Set orientation
      pose->mutable_orientation()->set_x(pose_stamped.pose.orientation.x);
      pose->mutable_orientation()->set_y(pose_stamped.pose.orientation.y);
      pose->mutable_orientation()->set_z(pose_stamped.pose.orientation.z);
      pose->mutable_orientation()->set_w(pose_stamped.pose.orientation.w);
    }
    
    // Publish to Gazebo
    if (gz_pub_.Publish(gz_path))
    {
      RCLCPP_INFO(this->get_logger(), "Published %d waypoints to Gazebo", gz_path.pose_size());
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to publish path to Gazebo");
    }
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  ignition::transport::Node gz_node_;
  ignition::transport::Node::Publisher gz_pub_;
};

}  // namespace gazebo_ros_actor_plugin

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gazebo_ros_actor_plugin::PathBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
