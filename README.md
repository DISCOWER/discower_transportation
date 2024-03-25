# Load Transportation Simulator (DISCOWER)
We present a simulator in Gazebo to simulate the Multi-agent load transportation package in the microgravity environment. 

## Required Installation
- ROS 2 Foxy
- Casadi
- Gazebo ROS2 Control plugin
- Gazebo Plugins

## Running Simulation
1. Clone the repository:
```
git clone git@github.com:pSujet/discower_transportation.git
```
2. Build the workspace:
```
colcon build --symlink-install
source install/setup.bash 
```
3. Start Gazebo simulation
```
ros2 launch discower_transportation launch_sim.launch.py 
```
4. Start PWM controller
```
ros2 run discower_transportation start_pwm_controller.py 
```
5. Run the Example controller.
This controller is a simple controller that moves the load forward.

```
ros2 run discower_transportation example_controller.py 
```
## Restart Simulation
We can reset the simulator by stopping example_controller and pwm_controller. Next, pressing ctrl + r in the Gazebo window. Then we can restart the cable position by launching the following command line:
```
ros2 launch discower_transportation set_initial.launch.py 
```

## Simulator
There are two robots and one load as shown in Fig. 1. The load is connected to the robots by cables. Each robot has four thruster pairs and is controlled by the PWM controller. Body wrench of each robot is calculated by the following equation:
$$
\begin{align}
    \bm{F} &= \bm{D} \bm{u} =
    \begin{bmatrix}
        F_x \\ F_y 
    \end{bmatrix} = 
    \begin{bmatrix}
    1 & 1 & 0 & 0 \\
    0 & 0 & 1 & 1
    \end{bmatrix}
    \begin{bmatrix}
    u_1 \\ u_2 \\ u_3 \\ u_4
    \end{bmatrix}  \\
    \bm{\tau} &= \bm{L}\bm{u} =
    \begin{bmatrix}
        \tau_z
    \end{bmatrix} = 
    l_{arm}\begin{bmatrix}
    1 & -1 & 1 & -1
    \end{bmatrix}
    \begin{bmatrix}
    u_1 \\ u_2 \\ u_3 \\ u_4
    \end{bmatrix}
\end{align}
$$

<div align="center">
<img src="fig/simulator.png" width="300">

**Fig. 1** Robot simulator
</div>



