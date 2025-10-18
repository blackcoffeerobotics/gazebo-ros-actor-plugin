#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion


class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher_node')
        
        # Create the publisher with topic "/cmd_path" and message type "Path"
        self.publisher = self.create_publisher(Path, '/cmd_path', 10)
        
        # Create a timer to publish periodically (every 2 seconds)
        self.timer = self.create_timer(2.0, self.publish_path)
        
        self.get_logger().info('Path publisher node started')
        
    def publish_path(self):
        # Create the Path message
        path_msg = Path()
        
        # Set the header for the Path message
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        
        # Create the PoseStamped messages for each waypoint
        num_waypoints = 10  # Number of waypoints on the path
        for i in range(num_waypoints):
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            
            angle = i * (2 * math.pi / num_waypoints)
            pose.pose.position = Point(
                x=2.0 * math.cos(angle), 
                y=2.0 * math.sin(angle), 
                z=0.0)
            
            # Calculate orientation (facing tangent to the circle)
            tangent_angle = angle + (math.pi / 2.0)
            
            # Convert to quaternion
            quat = self.euler_to_quaternion(0, 0, tangent_angle)
            pose.pose.orientation = Quaternion(
                x=quat[0],
                y=quat[1],
                z=quat[2],
                w=quat[3])
            
            path_msg.poses.append(pose)
        
        # Publish the Path message to the "/cmd_path" topic
        self.publisher.publish(path_msg)
        self.get_logger().info(f'Published path with {num_waypoints} waypoints')
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles to quaternion
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = [0] * 4
        q[0] = sr * cp * cy - cr * sp * sy  # x
        q[1] = cr * sp * cy + sr * cp * sy  # y
        q[2] = cr * cp * sy - sr * sp * cy  # z
        q[3] = cr * cp * cy + sr * sp * sy  # w
        
        return q


def main(args=None):
    rclpy.init(args=args)
    
    path_publisher = PathPublisher()
    
    try:
        rclpy.spin(path_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        path_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
