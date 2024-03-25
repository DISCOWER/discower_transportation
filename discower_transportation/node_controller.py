#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench,Vector3
import numpy as np
import time
from std_msgs.msg import Float32MultiArray

class ControllerNode(Node):
    '''
    This class is used to create a node for the MPC controller for the robot.
    The node will subscribe to the robot and load pose and publish the control input to the robot.
    '''
    def __init__(self,model=None,controller=None,init=None,simulation_time=None,CONTROLLER_ENABLE = False,PWM_ENABLE=True):
        '''
        Create subscriber, publisher and data logger
        '''
        super().__init__('controller_node')

        # -- Extract necessary parameter
        self.dt = model.dt
        self.n = model.n
        self.m = model.m
        self.model = model
        self.controller = controller
        self.i_dt = 0                                                               # counter for logging data

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
        self.publishing_rate = 10.0                                             # 10 Hz publishing rate (MPC rate)
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

        ###########################################################################
        ## Create data logger
        ###########################################################################

        # -- Initialized data 
        self.t_sim = simulation_time
        self.n_sim = int(self.t_sim/self.dt)
        self.x0 = init['x0']
        self.x_warm = init['x_warm']
        self.u_warm = init['u_warm']
        self.xref_array = init['xref']   
        SLEEP = True
        if SLEEP:
            self.sleep_time = 5
        else:
            self.sleep_time = 0
        self.sleep_time_count = 0 

        # -- Data logger
        self.x_Log = np.zeros((self.n, self.n_sim+1))                               # system states
        self.x_pred_Log = np.zeros((self.n_sim+1, self.n, self.controller.Nt+1))    # system prediction states
        self.u_Log = np.zeros((self.m, self.n_sim))                                 # control input
        self.cost_Log = np.zeros((1,self.n_sim))                                    # running cost
        self.compute_time_Log = np.zeros((1,self.n_sim))                            # computation time
        self.xref_Log = np.zeros((self.n, self.n_sim+1))                            # reference point
        self.trigger_Log = np.zeros((self.xref_array.shape[1]))                     # trigger time
        self.direct_Log = np.zeros((2,self.n_sim))                                  # tension in cable
        self.err_x_Log = np.zeros((2, self.n_sim+1))                                # position error (x,y)
        self.err_the_Log = np.zeros((1, self.n_sim+1))                              # orientation error (theta)

        # -- Initialized values
        self.x_Log[:,[0]] = self.x0                                                 # add x0 to the logger
        self.point = 0                                                              # set first reference point index
        self.xref = self.xref_array[:,[self.point]]                                 # get first reference point from the array
        self.xref_Log[:,[0]] = self.xref                                            # add first reference point to the logger
        self.err_x_Log[:,[0]] = self.xref[12:14] - self.x0[12:14]                   # add initial position error to the logger
        self.err_the_Log[:,[0]] = self.xref[16] - self.x0[16]                       # add initial orientation error to the logger
        
        # -- Prevent missing initial value issue by setting temporary value
        self.robot1_pose = np.vstack([0,0.9,0,0,0,0])                               # set initial position for robot 1
        self.robot2_pose = np.vstack([0,-0.9,0,0,0,0])                              # set initial position for robot 2
        self.load_pose = np.vstack([0,0,0,0,0,0])                                   # set initial position for load

    def load_odometry_callback(self, msg):
        '''
        Get relevant information from the Odometry message and update the current pose of the load
        '''
        self.load_pose = self.pose_msg_handle_load(msg)

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
        print(">> time: "+ str(self.timer_period*self.i_dt)+"s")   
        self.update_reference_point()                                             # check and update reference point
        
        # -- Check the last simultion step and stop controller (check before solving MPC problem for the last step)
        isFinished = self.i_dt > self.n_sim
        if isFinished:
            self.get_logger().info("Node is shutting down.")
            self.on_shutdown()
            self.destroy_node()
            rclpy.shutdown()   

        self.solve_mpc()                                                          # solve MPC and get control input in u_Log

        # -- Get body force from control input
        F1 = np.array(self.model.D1) @ self.u_Log[:,[self.i_dt]][0:4]             # input force in body frame (robot 1)
        T1 = np.array(self.model.L1) @ self.u_Log[:,[self.i_dt]][0:4]             # input torque in body frame (robot 1)
        F2 = np.array(self.model.D2) @ self.u_Log[:,[self.i_dt]][4:8]             # input force in body frame (robot 2)
        T2 = np.array(self.model.L2) @ self.u_Log[:,[self.i_dt]][4:8]             # input torque in body frame (robot 2)  
        
        # -- Add equilibrium force if need
        b_vec = np.array(self.model.bF(xk = self.x_Log[:,self.i_dt+1], uk = self.u_Log[:,self.i_dt])['b'])
        force_equi = 0.0
        force_equi1 = force_equi*b_vec
        force_equi2 = force_equi*-b_vec

        # -- Create Wrench message
        actuation_robot1 = Wrench(force=Vector3(x=F1[0][0]+force_equi1[0][0], y=F1[1][0]+force_equi1[1][0], z=0.0), torque=Vector3(x=0.0, y=0.0, z=T1[0][0]))
        actuation_robot2 = Wrench(force=Vector3(x=F2[0][0]+force_equi2[0][0], y=F2[1][0]+force_equi2[1][0], z=0.0), torque=Vector3(x=0.0, y=0.0, z=T2[0][0]))

        # -- Without PWM controller
        if self.CONTROLLER_ENABLE:
            self.robot1_pub.publish(actuation_robot1)
            self.robot2_pub.publish(actuation_robot2)

        # -- With PWM controller
        if self.PWM_ENABLE:
            input_robot_1 = Float32MultiArray(data=self.u_Log[:,[self.i_dt]][0:4])          # send control input of robot 1
            input_robot_2 = Float32MultiArray(data=self.u_Log[:,[self.i_dt]][4:8])          # send control input of robot 2
            # -- example message
            # input_robot_1 = Float32MultiArray(data=[0.1,0.1,0.5,0.5])          # send control input of robot 1
            # input_robot_2 = Float32MultiArray(data=[0.1,0.1,-0.5,-0.5])          # send control input of robot 2

            # -- publish message
            self.robot1pwm_pub.publish(input_robot_1)
            self.robot2pwm_pub.publish(input_robot_2)

            # # -- Get body force from control input
            # F1 = np.array(self.model.D1) @ self.u_Log[:,[self.i_dt]][0:4]             # input force in body frame (robot 1)
            # T1 = np.array(self.model.L1) @ self.u_Log[:,[self.i_dt]][0:4]             # input torque in body frame (robot 1)
            # F2 = np.array(self.model.D2) @ self.u_Log[:,[self.i_dt]][4:8]             # input force in body frame (robot 2)
            # T2 = np.array(self.model.L2) @ self.u_Log[:,[self.i_dt]][4:8]             # input torque in body frame (robot 2)

            # # -- Create Wrench message
            # actuation_robot1 = Wrench(force=Vector3(x=F1[0][0], y=F1[1][0], z=0.0), torque=Vector3(x=0.0, y=0.0, z=T1[0][0]))
            # actuation_robot2 = Wrench(force=Vector3(x=F2[0][0], y=F2[1][0], z=0.0), torque=Vector3(x=0.0, y=0.0, z=T2[0][0]))
            # self.robot1_pub.publish(actuation_robot1)
            # self.robot2_pub.publish(actuation_robot2)

        # -- Move index for data logging
        self.i_dt += 1
        
    def update_reference_point(self):
        '''
        Check error tolerance and update reference point
        '''
        # -- Get current pose of the system and stack in to a single vector 
        self.x_Log[:,[self.i_dt]] = np.vstack((self.robot1_pose,self.robot2_pose,self.load_pose)) 

        # -- Compute error for position and orientation      
        self.err_x_Log[:,[self.i_dt]] = self.xref[12:14] - self.x_Log[12:14,[self.i_dt]]
        self.err_the_Log[:,[self.i_dt]] = self.xref[16] - self.x_Log[16,[self.i_dt]]
        
        # -- Check tolerance error and update reference point
        position_trigger_error = 0.03#0.05 #0.01
        angle_trigger_error = 0.05#0.1        
        # position_tolerance = np.square(self.err_x_Log[:,[self.i_dt]]).sum(axis=0)
        # orientation_tolerance = np.abs(self.err_the_Log[:,[self.i_dt]])
        if np.linalg.norm(self.err_x_Log[:,[self.i_dt]]) <= position_trigger_error and abs(self.err_the_Log[:,[self.i_dt]]) <= angle_trigger_error:       # 0.03 0.02 quaternion     # 0.002 0.01
            self.sleep_time_count = self.sleep_time_count + self.dt            
            print('>>------------Target point: ' + str(self.point+1)+' ------------<<')
            print("Time count: "+str(self.sleep_time_count))
            if self.sleep_time_count >= self.sleep_time:
                self.sleep_time_count = 0
                if self.point == self.xref_array.shape[1]-1:
                    print('>>------------ FINAL POINT REACHED------------<<')
                else:
                    self.point = self.point + 1  
                    self.xref = self.xref_array[:,[self.point]]              
                    self.trigger_Log[self.point] = (self.i_dt)*self.dt                  # log trigger point for plotting     
        else:
            self.sleep_time_count = 0
        self.xref_Log[:,[self.i_dt]] = self.xref                                    # log reference point for plotting
    
    def solve_mpc(self):
        '''
        Solve MPC problem and get control input
        '''
        t_start = time.perf_counter() 
        self.u_Log[:,[self.i_dt]],x_temp,u_temp = self.controller.solve_mpc(self.x_Log[:,self.i_dt],self.xref,self.x_warm,self.u_warm)
        t_stop = time.perf_counter()
        time_elapse = t_stop-t_start
        self.compute_time_Log[:,[self.i_dt]] = time_elapse                          # log computation time for plotting
        print("Compute time: "+ str(time_elapse)) 
        self.x_pred_Log[self.i_dt,:,:] = x_temp
        # print("x_pred: "+ str(x_temp))
        # print("u_pred: "+ str(u_temp))
        # -- Create initial warmup states for solver by remove the initial element and copy the last element 
        self.x_warm = np.hstack((x_temp[:,1:],np.reshape(x_temp[:,-1],(-1,1))))
        self.u_warm = u_temp[:,0:]

        # -- Compute temporary variables
        n1_temp = self.model.n1F(self.x_Log[:,[self.i_dt]], self.u_Log[:,[self.i_dt]])          # unit vector from load to robot 1
        n2_temp = self.model.n2F(self.x_Log[:,[self.i_dt]], self.u_Log[:,[self.i_dt]])          # unit vector from load to robot 2
        Tv1_temp = self.model.tension1(self.x_Log[:,[self.i_dt]], self.u_Log[:,[self.i_dt]])    # cable 1 tension force vector 
        Tv2_temp = self.model.tension2(self.x_Log[:,[self.i_dt]], self.u_Log[:,[self.i_dt]])    # cable 2 tension force vector 

        # -- Compute running cost 
        self.cost_Log[:,[self.i_dt]] = self.controller.running_cost(self.x_Log[:,[self.i_dt]],self.xref,self.controller.Q,self.u_Log[:,[self.i_dt]],self.controller.R)
                         
        # -- Compute tension force
        direct1_temp = n1_temp.T @ Tv1_temp 
        direct2_temp = n2_temp.T @ Tv2_temp 
        self.direct_Log[:,[self.i_dt]] = np.vstack([direct1_temp,direct2_temp])

        input_cost = self.controller.input_cost(self.x_Log[:,[self.i_dt]],self.u_Log[:,[self.i_dt]],self.controller.R)
        print("Error: " + str(self.controller.error(self.x_Log[:,[self.i_dt]],self.xref)))
        print("MPC cost: "+ str(self.cost_Log[:,[self.i_dt]][0][0]))
        print("Input cost: " + str(input_cost))
        print("State cost: " + str(self.cost_Log[:,[self.i_dt]][0][0]-input_cost))
        # print("MPC cost: "+ str(self.cost_Log[:,[self.i_dt]][0][0]))
        # print("Error translation: "+str(np.linalg.norm(self.err_x_Log[:,[self.i_dt]])))
        # print("Error rotation: "+str(abs(self.err_the_Log[:,[self.i_dt]][0][0])))

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
    
    def pose_msg_handle_load(self,msg):
        '''
        Get Odometry message and pass only necessary data for planar coordinate (x,y,xdot,ydot,theta,thetadot)
        '''
        pose_x = msg.pose.pose.position.x
        pose_y = msg.pose.pose.position.y
        orientation_z = msg.pose.pose.orientation.z
        orientation_w = msg.pose.pose.orientation.w
        theta = self.quat2yaw(orientation_z,orientation_w)
        linear_x = msg.twist.twist.linear.x+0.0215
        linear_y = msg.twist.twist.linear.y-0.0215
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
        
        # -- change range from [-pi,pi] to [0,2pi]
        # if yaw < 0:
        #     return yaw + 2*np.pi
        # else:
        #     return yaw
        return yaw
        
    def construct_data(self):
        '''
        Wrap data into dictionary
        '''
        data = {}
        # -- Add data in structure
        data['x'] = self.x_Log
        data['u'] = self.u_Log
        data['xref_array'] = self.xref_array
        data['xref'] = self.xref_Log
        data['err_x'] = self.err_x_Log
        data['err_the'] = self.err_the_Log
        data['cost'] = self.cost_Log
        data['compute_time'] = self.compute_time_Log
        data['x_pred'] = self.x_pred_Log
        data['t_sim'] = self.t_sim
        data['n_sim'] = self.n_sim
        data['trigger'] = self.trigger_Log
        data['direct'] = self.direct_Log
        data['ulim'] = self.controller.ulim
        self.data = data

    def on_shutdown(self):
        '''
        Save data, stop controller and shutdown node
        '''        
        self.get_logger().info("Stop controller")    
        self.construct_data()    
        # -- Send zero command to the robot
        wrench_msg = Wrench(force=Vector3(x=0.0, y=0.0, z=0.0), torque=Vector3(x=0.0, y=0.0, z=0.0))
        self.load_pub.publish(wrench_msg)
        self.robot1_pub.publish(wrench_msg)
        self.robot2_pub.publish(wrench_msg)
        self.get_logger().info("Controller is stopped")

def main():
    rclpy.init()
    node = ControllerNode()
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
