# FYPRobot
FYP 2024 semester 1 repo

### File structure:
#### custom_ros_arduino_bridge:
This is a modfied version of `ros_arduino_bridge` to suit our particular motor driver. The code which is used in this document is in the firmware folder. This code is flashed to the arduino via Arduino IDE. 

Pinout for Arduino:
| Left Encoder  | Right Encoder | Left SPD    | Right SPD   | Left mode   | Right mode  |
| ------------- | ------------- |-------------|-------------|-------------|-------------|
| 2,3           | A4,A5         | 9           | 5           |10           | 6           |

----------------------------------------------------------------------------------------------------------------------------------
## Basic terminal commands:
### sourcing workspace
Needs to be done in every terminal from the workspace directory if not it will not be able to find the python scripts.
```
source install/local_setup.bash
```
### Source ros2 install:
This will need to be done in every terminal unless you add this line to the bottom of home/.bashrc
```
source /opt/ros/humble/setup.bash
```
### Build package
This must be done every time any change is made to the code. 
```
colcon build
```
----------------------------------------------------------------------------------------------------------------------------------
### Running the example test script:
This script runs RVIZ sim of the robot and also starts the arduino micro bridge communicating over serial
```
ros2 launch diffdrive_arduino diffbot.launch.py
```
Teleop topic rebind:
This is used to debug and test the manual driving and motor controller. Teleop uses cmd_vel by default as the topic, so as shown in the below the target topic is rebound to /ddd/cmd_vel.w
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/ddd/cmd_vel
```
----------------------------------------------------------------------------------------------------------------------------------
### Needed Software and packages
#### libserial
Needed for communication to the Arduino:
```
sudo apt install libserial-dev
```
#### Xacro:
```
sudo apt install ros-humble-xacro
```
#### Gazebo:
Simulation software to debug:
```
sudo apt install ros-humble-gazebo-ros-pkgs
```
#### ROS2 control:
```
sudo apt install ros-humble-ros2-control
```
#### ROS2 controllers:
```
sudo apt ros-foxy-ros2-controllers
```
