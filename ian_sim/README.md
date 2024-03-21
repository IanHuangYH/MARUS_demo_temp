# Seaclear Unity Simulator

## Description
Seaclear simulator in Unity built on the MARUSimulator (https://github.com/MARUSimulator)

## To Do
- Model doesn't interact with waves
- explore other installation installation methods (currently only for Nvidia GPUs in Docker)
- tweak sim parameters
- connect with all other simulation packages

# Installation
Have a look at the [marus-example wiki](https://github.com/MARUSimulator/marus-example/wiki/2.-Installation) for additional information
## Install Docker (only for Nvidia GPU!)

> ### Requirements

- Make sure you have Nvidia drivers installed (>=510). That can be validated using nvidia-smi command in terminal.

- Install Nvidia-container-toolkit (>= 1.12.1). How-to can be found on link: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html.

- Make sure your ssh key is added to GitHub! (for cloning the marus-docker repo)

> ### Building Docker image

Clone marus-docker
```
git clone git@github.com:MARUSimulator/marus-docker.git
```
In the marus-docker folder run:
```
docker build -t marus_docker . --build-arg ssh_prv_key="$(cat ~/.ssh/id_rsa)"  --build-arg ssh_pub_key="$(cat ~/.ssh/id_rsa.pub)"
```
This can take few minutes to build. Note: if not using default id_rsa key for github, mofify above command.

Create and run the Docker image:
```
xhost +

docker run  --name marus_container -v /tmp/.X11-unix:/tmp/.X11-unix  --gpus all  --runtime nvidia -e DISPLAY=$DISPLAY  --privileged -it marus_docker /bin/bash
```


Now you have docker container with marus-example and ROS backend installed on Ubuntu 20.04.
(More information: https://github.com/MARUSimulator/marus-example/wiki/2.-Installation)

## Import seaclear_unity_sim (in Docker)
Clone the seaclear_unity_sim (this may take a while)

```
git clone --recurse-submodules https://gitlab.cc-asp.fraunhofer.de/seaclear-project/tum/simulation/seaclear_unity_sim.git
```
Open unityhub (if prompted to install a Unity Editor you can *Skip Installation* for now)

```
unityhub
```
Now add a Personal license and add the seaclear_unity_sim project (install the recommended Unity Editor version 2021.3.33f1)

## ROS

> **ROS2**

* The ROS2 workspace was already created

> **ROS1**

* Install ros noetic & catkin

http://wiki.ros.org/noetic/Installation/Ubuntu

https://catkin-tools.readthedocs.io/en/latest/installing.html
* Create your workspace
    ```
    cd ~
    mkdir -p catkin_ws/src && cd catkin_ws/src
    ```
* Clone the grpc adapter (to connect Unity and ROS) and its dependencies
    ```
    git clone --recurse-submodules https://github.com/MARUSimulator/grpc_ros_adapter.git
    ```
    ```
    git clone https://github.com/labust/uuv_sensor_msgs.git
    ```
* An initial sourcing of your ros installation might be required (otherwise the ros2 workspace is sourced)
    ```
    source /opt/ros/noetic/setup.bash
    ```
* Build your workspace
    ```
    catkin_build
    ```
* Source your workspace
    ```
    source ~/catkin_ws/devel/setup.bash
    ```

# Usage
## Running Docker
```
xhost + (has to be run after every reboot of the host machine)
docker start marus_container
```
To enter the container (can be run in as many terminals as desired):
```
docker exec -it marus_container /bin/bash
```
## Running the simulator
Open UnityHub in terminal by executing:
```
unityhub
```
Open the Project and the desired scene (e.g. colrov_ocean.unity)

A tutorial featuring the basic functionalities can be found in the the [marus-example wiki](https://github.com/MARUSimulator/marus-example/wiki/3.-Quick-start)

## Basic features

### Open/Close gripper
Go to Gripper and set Open_gripper to 1 (open) or 0 (close)

* Currently gravity for gripper is deactivated and only joint friction seems to have an effect

### Drive
1. make sure you have the grpc adapter running and the pwm_out topic is visible
2. publish the pwm command over ros:
```
rostopic pub /colrov_skid/pwm_out std_msgs/Float32MultiArray "layout:
  dim:
  - label: ''
    size: 2
    stride: 0
  data_offset: 0
data:
- -1 
- -1
- 0
- 0
- 0
- 0
" -r15
```
* "-r15" specifies the frequency 

* the 6 values represent the 6 thrusters in order (0-5)

* e.g. to go down
- 0 
- 0
- 0
- 0
- -1
- -1

* go forward (broken only drives in circle)

- -1 
- -1
- 0
- 0
- 0
- 0


## Connecting to ROS
To connect to ROS, open the container in another terminal:
```
docker exec -it marus_container /bin/bash
```
* ROS2:
    ```
    ros2 launch grpc_ros_adapter ros2_server_launch.py
    ```
* ROS1:
    ```
    roslaunch grpc_ros_adapter launch_server.launch
    ```
**Be aware which workspace you source**