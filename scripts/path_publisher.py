#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseArray, Pose


class PoseArrayPublisher(Node):
    def __init__(self):
        super().__init__('pose_array_publisher_node')
        
        # Create the publisher with topic "/cmd_pose_array" and message type "PoseArray"
        self.publisher1 = self.create_publisher(PoseArray, '/cmd_path1', 10)
        self.publisher2 = self.create_publisher(PoseArray, '/cmd_path2', 10)
        self.publisher3 = self.create_publisher(PoseArray, '/cmd_path3', 10)
        self.publisher4 = self.create_publisher(PoseArray, '/cmd_path4', 10)

        # Create a timer to publish once after a short delay (to ensure subscriber is ready)
        self.timer = self.create_timer(1.0, self.publish_pose_array)
        self.published = False
        
        self.get_logger().info('PoseArray publisher node started')
        
    def publish_pose_array(self):

        #pedestrian1
        # pedestrian1
        pose_array_msg1 = PoseArray()
        pose_array_msg1.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg1.header.frame_id = "map"

        # waypoint 1
        pose1 = Pose()
        pose1.position = Point(x=0.0, y=-3.0, z=0.0)
        pose1.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        pose_array_msg1.poses.append(pose1)

        # waypoint 2
        pose1 = Pose()
        pose1.position = Point(x=0.0, y=-2.0, z=0.0)
        pose1.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        pose_array_msg1.poses.append(pose1)

        # waypoint 3
        pose1 = Pose()
        pose1.position = Point(x=-3.0, y=-2.0, z=0.0)
        pose1.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        pose_array_msg1.poses.append(pose1)

        # waypoint 4
        pose1 = Pose()
        pose1.position = Point(x=-3.0, y=-3.0, z=0.0)
        pose1.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        pose_array_msg1.poses.append(pose1)





        #pedestrian2
        pose_array_msg2 = PoseArray()
        pose_array_msg2.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg2.header.frame_id = "map"

        # waypoint 1
        pose2 = Pose()
        pose2.position = Point(x=-3.0, y=0.0, z=0.0)
        pose2.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        pose_array_msg2.poses.append(pose2)

        # waypoint 2
        pose2 = Pose()
        pose2.position = Point(x=-2.0, y=-3.0, z=0.0)
        pose2.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        pose_array_msg2.poses.append(pose2)

        # waypoint 3
        pose2 = Pose()
        pose2.position = Point(x=3.0, y=-2.0, z=0.0)
        pose2.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        pose_array_msg2.poses.append(pose2)

        # waypoint 4
        pose2 = Pose()
        pose2.position = Point(x=3.0, y=-3.0, z=0.0)
        pose2.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        pose_array_msg2.poses.append(pose2)




        #pedestrian3
        pose_array_msg3 = PoseArray()
        pose_array_msg3.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg3.header.frame_id = "map"

        # waypoint 1
        pose3 = Pose()
        pose3.position = Point(x=-3.0, y=0.0, z=0.0)
        pose3.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        pose_array_msg3.poses.append(pose3)

        # waypoint 2
        pose3 = Pose()
        pose3.position = Point(x=-2.0, y=-3.0, z=0.0)
        pose3.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        pose_array_msg3.poses.append(pose3)

        # waypoint 3
        pose3 = Pose()
        pose3.position = Point(x=3.0, y=-2.0, z=0.0)
        pose3.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        pose_array_msg3.poses.append(pose3)

        # waypoint 4
        pose3 = Pose()
        pose3.position = Point(x=3.0, y=-3.0, z=0.0)
        pose3.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        pose_array_msg3.poses.append(pose3)




        #pedestrian4
        pose_array_msg4 = PoseArray()
        pose_array_msg4.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg4.header.frame_id = "map"

        # waypoint 1
        pose4 = Pose()
        pose4.position = Point(x=-3.0, y=0.0, z=0.0)
        pose4.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        pose_array_msg4.poses.append(pose4)

        # waypoint 2
        pose4 = Pose()
        pose4.position = Point(x=-2.0, y=-3.0, z=0.0)
        pose4.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        pose_array_msg4.poses.append(pose4)

        # waypoint 3
        pose4 = Pose()
        pose4.position = Point(x=3.0, y=-2.0, z=0.0)
        pose4.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        pose_array_msg4.poses.append(pose4)

        # waypoint 4
        pose4 = Pose()
        pose4.position = Point(x=3.0, y=-3.0, z=0.0)
        pose4.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        pose_array_msg4.poses.append(pose4)



        # Only publish once
        if self.published:
            return
        
        # Create the PoseArray message
        pose_array_msg = PoseArray()
        
        # Set the header for the PoseArray message
        pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg.header.frame_id = "map"
        
        # Create the Pose messages for each waypoint
        #num_waypoints = 10  # Number of waypoints on the path
        #for i in range(num_waypoints):
        #    pose = Pose()
        #    
        #    angle = i * (2 * math.pi / num_waypoints)
        #    pose.position = Point(
        #        x=2.0 * math.cos(angle), 
        #        y=2.0 * math.sin(angle), 
        #        z=0.0)
            
            # Calculate orientation (facing tangent to the circle)
        #    tangent_angle = angle + (math.pi / 2.0)
            
            # Convert to quaternion
        #    quat = self.euler_to_quaternion(0, 0, tangent_angle)
        #    pose.orientation = Quaternion(
        #        x=quat[0],
        #        y=quat[1],
        #        z=quat[2],
        #        w=quat[3])
            
        #    pose_array_msg.poses.append(pose)
        
        # Publish the PoseArray message to the "/cmd_pose_array" topic
        self.publisher1.publish(pose_array_msg1)
        self.publisher2.publish(pose_array_msg2)
        self.publisher3.publish(pose_array_msg3)
        self.publisher4.publish(pose_array_msg4)

        #self.get_logger().info(f'Published PoseArray with {num_waypoints} poses')
        
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
