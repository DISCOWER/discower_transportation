#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench,Vector3
import numpy as np
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import casadi as ca

class PWMNode(Node):
    '''
    Node class to imitate the pwm behavior, for example if the controller recceive 0.5 it will 
    send maximum thrust for 50% of duty cycle to the robot
    '''
    def __init__(self,ulim,CONTROLLER_ENABLE=True,LOG_INPUT=False):
        super().__init__('controller_node')
        self.LOG_INPUT = LOG_INPUT
        self.CONTROLLER_ENABLE = CONTROLLER_ENABLE

        ###########################################################################
        ## Create subscriber and publisher
        ###########################################################################

        # -- Create subscriber to get control input from the MPC controller
        self.robot1_sub = self.create_subscription(Float32MultiArray,'/robot1/pwm',self.robot1_pwm_callback,20)
        self.robot2_sub = self.create_subscription(Float32MultiArray,'/robot2/pwm',self.robot2_pwm_callback,20)
        
        # -- create publisher to publish the wrench to the robot
        self.robot1_pub = self.create_publisher(Wrench,'/robot1/wrench', 20)
        self.robot2_pub = self.create_publisher(Wrench,'/robot2/wrench', 20)

        # -- Create the timer to publish control input at specific rate
        self.timer_period = 1/2000.0                       # time period between each publish in form of 1/f Hz publishing rate
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        ###########################################################################
        ## Set and initialize parameters
        ###########################################################################

        # -- PWM parameters
        self.duty_cycle = 0.1                                       # period of time for one duty cycle in second
        self.ulim = ulim                                            # maximum thrust    
        self.duty_pwm_step = self.duty_cycle/self.timer_period      # number of steps for one duty cycle    
        self.i_step = 0                                             # counter for PWM signal (step in duty cylcle) 

        # -- Initialize values
        self.u = np.zeros((8,1))                                    # control input
        self.robot1_pwm = np.zeros((4,1))                           # pwm input robot 1
        self.robot2_pwm = np.zeros((4,1))                           # pwm input robot 2
        self.i_dt = 0                                               # counter for logging data

        # -- Log input for debugging
        if self.LOG_INPUT:
            self.run_time = 5
            self.run_time_step = int(self.run_time/self.timer_period)
            self.upwm_log = np.zeros((8,self.run_time_step))        # log pwm input
        
        # -- Thruster matrix
        self.D1 = ca.DM([[1,1,0,0],[0,0,1,1]])                               # thrust direction matrix robot 1
        self.L1 = ca.DM([[1,-1,1,-1]])*0.12                                  # thrust arm matrix robot 1
        self.D2 = ca.DM([[1,1,0,0],[0,0,1,1]])                               # thrust direction matrix robot 2
        self.L2 = ca.DM([[1,-1,1,-1]])*0.12                                  # thrust arm matrix robot 2
        
        self.get_logger().info("PWM controller is ready")

    def robot1_pwm_callback(self, msg):
        '''
        Get control input from MPC and translate in to PWM signal (0 or max) for the robot 1
        '''
        self.robot1_pwm = self.msg_handle(msg)
        self.i_step = 0
    
    def robot2_pwm_callback(self, msg):
        '''
        Get control input from MPC and translate in to PWM signal (0 or max) for the robot 2
        '''
        self.robot2_pwm = self.msg_handle(msg)
        self.i_step = 0

    def timer_callback(self):     
        '''
        Publish the Wrench to the robot according to the PWM signal
        ''' 
        if self.i_step == self.duty_pwm_step:
            self.i_step = 0

        # -- Converge pwm to wrench input
        u_pwm = np.vstack((self.robot1_pwm,self.robot2_pwm)) 
        for i in range(8):
            self.u[i] = self.pwm2input(u_pwm[i])     

        # -- Log input
        if self.LOG_INPUT:
            self.upwm_log[:,self.i_dt] = self.u[:,0]

        # -- Get body force from input        
        F1 = np.array(self.D1) @ self.u[0:4]             # input force in body frame (robot 1)
        T1 = np.array(self.L1) @ self.u[0:4]             # input torque in body frame (robot 1)
        F2 = np.array(self.D2) @ self.u[4:8]             # input force in body frame (robot 2)
        T2 = np.array(self.L2) @ self.u[4:8]             # input force in body frame (robot 2) 
        actuation_robot1 = Wrench(force=Vector3(x=F1[0][0], y=F1[1][0], z=0.0), torque=Vector3(x=0.0, y=0.0, z=T1[0][0]))
        actuation_robot2 = Wrench(force=Vector3(x=F2[0][0], y=F2[1][0], z=0.0), torque=Vector3(x=0.0, y=0.0, z=T2[0][0]))

        # -- Enable controller and publish the wrench
        if self.CONTROLLER_ENABLE:
            self.robot1_pub.publish(actuation_robot1)           # publish wrench to robot 1
            self.robot2_pub.publish(actuation_robot2)           # publish wrench to robot 2

        # -- Update counter
        self.i_step += 1                                        # update step in duty cycle
        self.i_dt += 1                                          # update counter for logging data

    def msg_handle(self,msg):
        '''
        Get PWM percentage from the input message (u/u_lim)
        '''
        u = np.reshape(np.array(msg.data)/self.ulim,(-1,1))
        return u

    def pwm2input(self,pwm):
        '''
        Convert PWM signal to input either 0 or max
        '''
        current_pwm_step = self.i_step%self.duty_pwm_step                # current step in duty cycle
        current_pwm = current_pwm_step/self.duty_pwm_step                # current pwm percentage
        if current_pwm >= np.abs(pwm):
            input = 0
        else:   
            # -- Check direction of input
            if pwm > 0:
                input = self.ulim
            else:
                input = -self.ulim
        return input
        
        
    def on_shutdown(self):
        '''
        Save data, stop controller and shutdown node
        '''       
        self.get_logger().info("Stop controller")   

        # -- Send zero command to the robot
        wrench_msg = Wrench(force=Vector3(x=0.0, y=0.0, z=0.0), torque=Vector3(x=0.0, y=0.0, z=0.0))
        self.robot1_pub.publish(wrench_msg)
        self.robot2_pub.publish(wrench_msg)

        if self.LOG_INPUT:
            t = np.linspace(0,self.run_time,self.run_time_step)
            plt.figure()
            plt.step(t,self.upwm_log[0,:], where='post')
            plt.grid()
            plt.figure()
            plt.step(t,self.upwm_log[1,:], where='post')
            plt.grid()
            plt.figure()
            plt.step(t,self.upwm_log[2,:], where='post')
            plt.grid()
            plt.figure()
            plt.step(t,self.upwm_log[3,:], where='post')
            plt.grid()
            plt.show()
        self.get_logger().info("Controller is stopped")

def main():
    rclpy.init()
    node = PWMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Perform cleanup tasks or any actions you want on keyboard interrupt
        node.get_logger().info("Keyboard interrupt received. Stop the controller")
    finally:
        # Clean up resources, release memory, or save data here if needed
        node.get_logger().info("Node is shutting down.")
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
