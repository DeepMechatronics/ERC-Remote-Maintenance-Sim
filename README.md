# ERC UR3 Simulation

This repository contains simulations of the Universal Robots UR3 robot created for the ERC competition. You can use docker with all requirements installed  ([Using Docker](#using-docker) section) or try to install them natively ([Install on the host system](#install-on-the-host-system) section).

UPDATE

The simulation includes the UR3 manipulator with an attached gripper, prepared especially for this year's competition. With MoveIt, you can plan the movement of both the robot and the gripper. An RGB camera was also installed above the gripper, the image of which can be used to detect ArUco tags. 
An example scene was also created. There are three panels on it. There is a button on the middle one with an ArUco tag above it. Next to the panel on the left side there is a box that will actually contain the IMU module. The ArUco tag is on it and on the left panel, in the place where it is to be attached . An inspection window consisting of a box and a lid has been mounted on the right-hand panel. One ArUco tag is on the lid and the other is on the box.


## Using Docker

---
**NOTE**

The commands in this section should be executed as the `root` user, unless you have configured docker to be [managable as a non-root user](https://docs.docker.com/engine/install/linux-postinstall/).

---

Make sure the [Docker Engine](https://docs.docker.com/engine/install/#server) is installed and the `docker` service is running:
```
systemctl start docker
```
Build the docker image by executing:
```
./build_erc.sh 
```
Create container:
If you are running the system with no dedicated GPU (not recommended), execute:
```
bash run_ur3_docker.bash
```
To use an integrated AMD/Intel Graphics card, run:
```
bash run_ur3_docker_AMD_Intel_GRaphics.bash
```
To use an Nvidia card, you need to previously install proprietary drivers and Nvidia Container Toolkit (https://github.com/NVIDIA/nvidia-docker). Next execute command:
``` 
bash run_ur3_docker_Nvidia.bash
```
Then, you can start container:
```
docker start ur3_simulation_erc
```
Then, you can run command in running container:
```
$ docker exec -it ur3_simulation_erc /bin/bash
```

Now you can run commands below in docker container.

UR3 simulation in Gazebo with MoveIt! and RViz GUI:
Simulation in Gazebo:
```
$ roslaunch simulation simulator.launch 
```
## Install on the host system

### Requirements

The simulation is mainly developed and tested on [Ubuntu 18.04 Bionic Beaver](https://releases.ubuntu.com/18.04/) with [ROS Melodic Morenia](http://wiki.ros.org/melodic/Installation/Ubuntu), so it is a recommended setup. 

For the simulation to work properly, you must install dependencies and download repository by running the following commands: 
``` 
sudo apt-get update && apt-get upgrade -y && apt-get install -y lsb-core g++
rosdep init && rosdep update
mkdir -p /catkin_ws/src
cd /catkin_ws/src && git clone https://github.com/Michal-Bidzinski/ERC_2021_simulator.git
apt install ros-melodic-industrial-core -y
rosdep update
rosdep install --from-paths /catkin_ws/src/ --ignore-src --rosdistro melodic -r -y
sudo apt install ros-melodic-teleop* -y
sudo apt install ros-melodic-joy* -y
sudo apt install ros-melodic-aruco-ros* -y
sudo apt install -y python-pip
sudo apt install -y python3-pip
cd /catkin_ws
catkin_make
source /opt/ros/melodic/setup.bash
source /catkin_ws/devel/setup.bash
```
### Run simulation
UR3 simulation in Gazebo with MoveIt! and RViz GUI:
Simulation in Gazebo:
```
$ roslaunch simulation simulator.launch 
```

### Cube with an ArUco marker
In ur_description/meshes/ there is a .dae file containing a cube with an aruco tag (photo maker582_small_margins.png). You can import it into the simulation to try to detect it with the camera mounted on the robot, and then move the manipulator to the marker (this is one of many possible scenarios). 

