#!/usr/bin/env python3

"""
Path bridge script that converts ROS2 nav_msgs/Path to Gazebo Pose_V message.
This script subscribes to a ROS2 nav_msgs/Path topic and republishes it to
Gazebo using a persistent gz topic publisher process.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import subprocess
import json
import threading


class PathBridgeNode(Node):
    """
    ROS2 node that bridges nav_msgs/Path to Gazebo Pose_V messages.
    Uses a persistent gz topic publisher subprocess.
    """

    def __init__(self):
        super().__init__('path_bridge_node')
        
        # Declare parameters
        self.declare_parameter('ros_path_topic', '/cmd_path')
        self.declare_parameter('gz_path_topic', '/cmd_path')
        
        # Get parameters
        self.ros_path_topic = self.get_parameter('ros_path_topic').value
        self.gz_path_topic = self.get_parameter('gz_path_topic').value
        
        # Create ROS2 subscriber
        self.path_sub = self.create_subscription(
            Path,
            self.ros_path_topic,
            self.path_callback,
            10
        )
        
        # Store latest path for publishing
        self.latest_path_json = None
        self.publisher_lock = threading.Lock()
        
        # Start a timer to periodically publish the latest path
        self.publish_timer = self.create_timer(0.1, self.publish_to_gazebo)  # 10 Hz
        
        self.get_logger().info('Path bridge node started')
        self.get_logger().info(f'  ROS2 topic: {self.ros_path_topic}')
        self.get_logger().info(f'  GZ topic: {self.gz_path_topic}')
        self.get_logger().info('  Publishing at 10 Hz when path is available')

    def path_callback(self, msg):
        """
        Callback for receiving nav_msgs/Path messages from ROS2.
        Stores the path for periodic publishing to Gazebo.
        
        Args:
            msg: nav_msgs/Path message
        """
        if len(msg.poses) == 0:
            self.get_logger().warn('Received empty path, ignoring')
            return
        
        self.get_logger().info(f'Received ROS path with {len(msg.poses)} waypoints')
        
        # Build the Pose_V message for Gazebo
        poses_list = []
        for pose_stamped in msg.poses:
            pose = pose_stamped.pose
            pose_dict = {
                'position': {
                    'x': pose.position.x,
                    'y': pose.position.y,
                    'z': pose.position.z
                },
                'orientation': {
                    'x': pose.orientation.x,
                    'y': pose.orientation.y,
                    'z': pose.orientation.z,
                    'w': pose.orientation.w
                }
            }
            poses_list.append(pose_dict)
        
        # Create the complete Pose_V message
        pose_v_msg = {'pose': poses_list}
        
        # Store the JSON for periodic publishing
        with self.publisher_lock:
            self.latest_path_json = json.dumps(pose_v_msg)
        
        self.get_logger().info(f'Path stored for publishing ({len(poses_list)} waypoints)')

    def publish_to_gazebo(self):
        """
        Timer callback that periodically publishes the latest path to Gazebo.
        This creates a persistent publisher effect.
        """
        with self.publisher_lock:
            if self.latest_path_json is None:
                return  # No path to publish yet
            
            msg_json = self.latest_path_json
        
        # Publish using gz topic command
        try:
            cmd = [
                'gz', 'topic', '-t', self.gz_path_topic,
                '-m', 'gz.msgs.Pose_V',
                '-p', msg_json
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=1)
            
            if result.returncode != 0 and result.stderr:
                self.get_logger().error(f'Failed to publish to Gazebo: {result.stderr}')
                
        except subprocess.TimeoutExpired:
            self.get_logger().warn('Timeout while publishing to Gazebo')
        except Exception as e:
            self.get_logger().error(f'Error publishing to Gazebo: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = PathBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
