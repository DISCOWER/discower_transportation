'''
This launch file is used to set the initial joint position (cable length) of the robot.
''' 
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_prefix
  
def generate_launch_description():
    package_name='discower_transportation' 
    pwm_controller_path = os.path.join(get_package_prefix(package_name),'lib',package_name,'start_pwm_controller.py')
    # Set initial joint position (cable length) of the robot
    initial_joint_msg = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub','-1', '/set_joint_trajectory', 'trajectory_msgs/msg/JointTrajectory',
        '{header: {frame_id: world}, joint_names: [anchor1_joint,anchor2_joint], \
         points: [{positions: [0.4999,0.4999],velocities: [0.0000, 0.0000],accelerations: [0.0000,0.0000]}]}'],
        shell=False,
    )
    pwm_controller = ExecuteProcess(
        cmd=['python3', 
             pwm_controller_path],
        output='screen'
    )


    return LaunchDescription([
        initial_joint_msg,
        pwm_controller,
    ])