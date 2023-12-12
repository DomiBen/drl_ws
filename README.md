# drl_ws

Installation instrucktions for setting up the mirobot simulation and services for Deep Reinforcement Learning

# Setup:
1. [Download and install ROS Noetic](http://wiki.ros.org/noetic/Installation)
2. Download and install MuJoCo 2.3.6 and mujoco-py similar to the [instructions](https://gist.github.com/saratrajput/60b1310fe9d9df664f9983b38b50d5da), DO NOT USE Anaconda!
3. Update your packages ``` sudo apt-get update ```
4. Install ROS controllers:``` sudo apt-get install ros-noetic-ros-controllers ```
5. Install catkin tools:``` sudo apt-get install python3-catkin-tools ```
6. Clone [mujoco_ws](https://github.com/DomiBlack2k/mujoco_ws) and [drl_ws](https://github.com/DomiBlack2k/drl_ws) in your home folder
7. For both catkin workspaces run catkin clean
8. For the mujoco_ws follow [these build instructions](https://github.com/ubi-agni/mujoco_ros_pkgs)https://github.com/ubi-agni/mujoco_ros_pkgs
9. Navigate into ~/drl_ws/hrl-kdl and follow the [README Instructions](https://github.com/DomiBlack2k/drl_ws/blob/main/src/hrl-kdl/README.md)https://github.com/DomiBlack2k/drl_ws/blob/main/src/hrl-kdl/README.md
10. Build the drl_ws using catkin build
11. Add the setup.bash files to your bashrc:
```
echo "source /opt/ros/noetic/setup.bash"
"source ~/drl_ws/devel/setup.bash"
"source ~/mujoco_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
# Starting the MuJoCo and Mirobot Server: 
```
roslaunch mirobot_mujoco_ros mirobot_mujoco.launch
```
Alternatively you can start the MuJoCO Server alone: 
```
roslaunch mirobot_mujoco_ros load_mirobot_model.launch
roslaunch mirobot_mujoco_ros bringup_mujoco_ros.launch
```
# Check if everything is running fine: 
```
rosservice list
```
should list /MirobotServer/SetXY Services
```
rostopic list
```
should list /endeffector_pose, /force, /torque etc.

# Check RL environment:
```
rosrun deep_reinforcemnte_learning env_test.py
```
Should run without Errors and move the Robot in MuJoCo
