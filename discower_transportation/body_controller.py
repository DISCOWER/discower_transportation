#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench,Vector3
import numpy as np
import time
from std_msgs.msg import Float32MultiArray

class BodyControllerNode(Node):
    '''
    This class is used to create a node for a simple controller for the robot.
    The node will subscribe to the robot and load pose and publish the control input to the robot.
    '''
    def __init__(self,CONTROLLER_ENABLE = False,PWM_ENABLE=True):
        '''
        Create subscriber, publisher and data logger
        '''
        super().__init__('controller_node')

        ###########################################################################
        ## Create subscriber and publisher
        ###########################################################################

        # -- Create subscriber to get robot pose in Odometry message (position, orientation and twist)
        self.load_sub = self.create_subscription(Odometry,'/load/pose',self.load_odometry_callback,20)
        self.robot1_sub = self.create_subscription(Odometry,'/robot1/pose',self.robot1_odometry_callback,20)
        self.robot2_sub = self.create_subscription(Odometry,'/robot2/pose',self.robot2_odometry_callback,20)

        # -- Create publisher to publish control input to the robot in Wrench message (Force and Torque)
        self.load_pub = self.create_publisher(Wrench, '/load/wrench', 20)
        self.robot1_pub = self.create_publisher(Wrench, '/robot1/wrench', 20)
        self.robot2_pub = self.create_publisher(Wrench, '/robot2/wrench', 20)

        # -- Create the timer to publish control input at specific rate
        self.publishing_rate = 10.0                                             # 10 Hz publishing rate (controller rate)
        self.timer_period = 1.0/self.publishing_rate                            # period between each message 0.1s 
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # -- Enable controller without pwm controller
        self.CONTROLLER_ENABLE = CONTROLLER_ENABLE

        # -- Enable pwm signal to pwm controller
        # This controller will send the control input in form of Float32MultiArray (len = 4) for each robot
        self.PWM_ENABLE = PWM_ENABLE
        if self.PWM_ENABLE:
            self.robot1pwm_pub = self.create_publisher(Float32MultiArray,'/robot1/pwm', 20)
            self.robot2pwm_pub = self.create_publisher(Float32MultiArray,'/robot2/pwm', 20)       

    def load_odometry_callback(self, msg):
        '''
        Get relevant information from the Odometry message and update the current pose of the load
        '''
        self.load_pose = self.pose_msg_handle(msg)

    def robot1_odometry_callback(self, msg):
        '''
        Get relevant information from the Odometry message and update the current pose of the robot1
        '''
        self.robot1_pose = self.pose_msg_handle(msg)
    
    def robot2_odometry_callback(self, msg):
        '''
        Get relevant information from the Odometry message and update the current pose of the robot2
        '''
        self.robot2_pose = self.pose_msg_handle(msg)

    def timer_callback(self):  
        '''
        Get current time and publish control input to robot1 and 2 
        '''   
        self.get_logger().info("Sending control input to the robot")

        # -- Create Wrench message
        actuation_robot1 = Wrench(force=Vector3(x=0.0, y=0.0, z=0.0), torque=Vector3(x=0.0, y=0.0, z=0.0))
        actuation_robot2 = Wrench(force=Vector3(x=0.0, y=0.0, z=0.0), torque=Vector3(x=0.0, y=0.0, z=0.0))
        
        # -- Without PWM controller
        if self.CONTROLLER_ENABLE:
            self.robot1_pub.publish(actuation_robot1)
            self.robot2_pub.publish(actuation_robot2)

        # -- With PWM controller
        if self.PWM_ENABLE:
            input_robot_1 = Float32MultiArray(data=[0.1,0.1,0.5,0.5])          # send control input of robot 1
            input_robot_2 = Float32MultiArray(data=[0.1,0.1,-0.5,-0.5])          # send control input of robot 2
            self.robot1pwm_pub.publish(input_robot_1)
            self.robot2pwm_pub.publish(input_robot_2)

    def pose_msg_handle(self,msg):
        '''
        Get Odometry message and pass only necessary data for planar coordinate (x,y,xdot,ydot,theta,thetadot)
        '''
        pose_x = msg.pose.pose.position.x
        pose_y = msg.pose.pose.position.y
        orientation_z = msg.pose.pose.orientation.z
        orientation_w = msg.pose.pose.orientation.w
        theta = self.quat2yaw(orientation_z,orientation_w)
        linear_x = msg.twist.twist.linear.x
        linear_y = msg.twist.twist.linear.y
        angular_z = msg.twist.twist.angular.z
        pose = np.vstack([pose_x,
                          pose_y,
                          linear_x,
                          linear_y,
                          theta,
                          angular_z])
        return pose
        
    
    def quat2yaw(self,qz,qw):
        '''
        Compute yaw angle from quaternion in radian
        '''
        yaw = np.arctan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz**2)) 
        return yaw

    def on_shutdown(self):
        '''
        Save data, stop controller and shutdown node
        '''        
        self.get_logger().info("Stop controller")    
        # -- Send zero command to the robot
        wrench_msg = Wrench(force=Vector3(x=0.0, y=0.0, z=0.0), torque=Vector3(x=0.0, y=0.0, z=0.0))
        self.load_pub.publish(wrench_msg)
        self.robot1_pub.publish(wrench_msg)
        self.robot2_pub.publish(wrench_msg)
        self.get_logger().info("Controller is stopped")

def main():
    rclpy.init()
    node = BodyControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Stop the controller")
    finally:
        # -- Clean up resources, release memory, and save data 
        node.get_logger().info("Node is shutting down.")
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
