import os
 
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
 
 
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
 
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
 
 
 
def generate_launch_description():
 
 
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
 
    package_name='discower_transportation' 
 
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
 
    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )
 
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot', "-x", "0.0", "-y", "0.0", "-z", "0.0"],
                        output='screen')
   
    # Set initial joint position (cable length) of the robot
    initial_joint_msg = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub','-1', '/set_joint_trajectory', 'trajectory_msgs/msg/JointTrajectory',
        '{header: {frame_id: world}, joint_names: [anchor1_joint,anchor2_joint], \
         points: [{positions: [0.4999,0.4999],velocities: [0.0, 0.0],accelerations: [0.0,0.0]}]}'],
        shell=False,
    )
    # ,{velocities: [0.0,0.0]},{accelerations: [0.0,0.0]}
    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        initial_joint_msg,
    ])