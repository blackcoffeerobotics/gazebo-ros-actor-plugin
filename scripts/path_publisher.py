#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseArray, Pose


class PoseArrayPublisher(Node):
    def __init__(self):
        super().__init__('pose_array_publisher_node')
        
        # Create the publisher with topic "/cmd_pose_array" and message type "PoseArray"
        self.publisher = self.create_publisher(PoseArray, '/cmd_path', 10)
        
        # Create a timer to publish once after a short delay (to ensure subscriber is ready)
        self.timer = self.create_timer(1.0, self.publish_pose_array)
        self.published = False
        
        self.get_logger().info('PoseArray publisher node started')
        
    def publish_pose_array(self):
        # Only publish once
        if self.published:
            return
        
        # Create the PoseArray message
        pose_array_msg = PoseArray()
        
        # Set the header for the PoseArray message
        pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg.header.frame_id = "map"
        
        # Create the Pose messages for each waypoint
        num_waypoints = 10  # Number of waypoints on the path
        for i in range(num_waypoints):
            pose = Pose()
            
            angle = i * (2 * math.pi / num_waypoints)
            pose.position = Point(
                x=2.0 * math.cos(angle), 
                y=2.0 * math.sin(angle), 
                z=0.0)
            
            # Calculate orientation (facing tangent to the circle)
            tangent_angle = angle + (math.pi / 2.0)
            
            # Convert to quaternion
            quat = self.euler_to_quaternion(0, 0, tangent_angle)
            pose.orientation = Quaternion(
                x=quat[0],
                y=quat[1],
                z=quat[2],
                w=quat[3])
            
            pose_array_msg.poses.append(pose)
        
        # Publish the PoseArray message to the "/cmd_pose_array" topic
        self.publisher.publish(pose_array_msg)
        self.get_logger().info(f'Published PoseArray with {num_waypoints} poses')
        
        # Mark as published and cancel the timer
        self.published = True
        self.timer.cancel()
        
        # Schedule shutdown
        self.get_logger().info('PoseArray published successfully. Shutting down...')
        self.create_timer(0.1, self.destroy_and_exit)
    
    def destroy_and_exit(self):
        self.destroy_node()
        raise SystemExit
    
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
    
    pose_array_publisher = PoseArrayPublisher()
    
    rclpy.spin(pose_array_publisher)
    
    pose_array_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
