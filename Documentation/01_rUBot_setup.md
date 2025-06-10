# **ROS2 rUBot setup**

The objectives of this section are:
- Setup the robot project in virtual environment for simulation
- Setup the robot project for real control
- Install needed interfaces

We have:
- Commercial **LIMO** robot

![](./Images/01_Setup/rUBot_Limo.png)

Webgraphy:
- TheConstruct: Build Your First ROS2 Based Robot https://www.robotigniteacademy.com/courses/309
- LIMO repository: https://github.com/agilexrobotics/limo_ros2/tree/humble
- LIMO Doc: https://github.com/agilexrobotics/limo_pro_doc/blob/master/Limo%20Pro%20Ros2%20Foxy%20user%20manual(EN).md
- LIMO bitbucket: https://bitbucket.org/theconstructcore/limo_robot/src/main/
- https://bitbucket.org/theconstructcore/workspace/projects/ROB
- TheConstruct image Humble-v3: https://hub.docker.com/r/theconstructai/limo/tags
- https://github.com/AntoBrandi/Self-Driving-and-ROS-2-Learn-by-Doing-Odometry-Control/tree/main
- https://github.com/AntoBrandi/Self-Driving-and-ROS-Learn-by-Doing-Odometry-Control
- https://github.com/AntoBrandi/Arduino-Bot/tree/humble


## **1. Setup the robot project in virtual environment for simulation**

a) For **simulation** Using TheConstruct interface, we will have to clone the github repository:

```shell
git https://github.com/Mattyete/ROS2_LIMO_ws.git
cd ROS2_LIMO_ws
colcon build
source install/local_setup.bash
```
- Add in .bashrc the lines:
````shell
export ROS_DOMAIN_ID=0
export TURTLEBOT3_MODEL=waffle
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GAZEBO_MODEL_PATH=/home/user/ROS2_LIMO_ws/src/my_robot_bringup/models:$GAZEBO_MODEL_PATH
source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /home/user/ROS2_rUBot_tutorial_ws/install/setup.bash
source /home/user/ROS2_LIMO_ws/install/setup.bash
#cd /home/user/ROS2_rUBot_tutorial_ws
cd /home/user/ROS2_LIMO_ws

````
- If the compilation process returns warnings on "Deprecated setup tools":
````shell
sudo apt install python3-pip
pip3 list | grep setuptools
pip3 install setuptools==58.2.0
````
- If the compilation process returns wardings on PREFIX_PATH:
````shell
unset COLCON_PREFIX_PATH
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
cd ~/ROS2_LIMO_ws
rm -rf build/ install/ log/
colcon build
````
- Open a new terminal to ensure the .bashrc is read again

## **2. Setup the robot project for real control**

Here we will review the Computer onboard used for each robot and the designed setup process.

The setup process is based on a custom Docker to properly interface with the ROS2 environment.

### **2.1. Setup the LIMO robot**

- Jetson Nano computer onboard
- Custom Dockerfile and docker-compose 

When the commercial LIMO robot is plugged on, the docker-compose-v3.yaml service is executed and the LIMO robot is ready to be controlled within the TheConstruct environment.

- Using a PC connected to the same network 
  - connect with robot within ssh using VScode
  - review the running containers
  - Attach a new VScode window on the Limo container
  - to see the ros topics type
    ````shell
    source /limo_entrypoint-v3.sh
    ros2 topic list
    ````
- Using the TheConstruct RRL service
  - Install the robot on your account (this is already done for you)
  - Connect to the robot and type in a new terminal
    ````shell
    ros2 topic list
    ````
