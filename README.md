# ERC UR3 Simulation

This repository contains simulations of the Universal Robots UR3 robot created for the ERC competition. You can use docker with all requirements installed  ([Using Docker](#using-docker) section) or try to install them natively ([Install on the host system](#install-on-the-host-system) section).

**UPDATE V2**

The gripper is controlled by publishing appropriate commands on topic /gripper_command. Message type: stdmsgs/String. To control the gripper, send the following commands:
•open
•semi_open
•semi_close
•close

The real gripper will also be steered this way. Due to the fact that the simulation does not fully reflect the reality, in particular the interaction between the gripped objects and the robot, the gripper in the semi-open and semi-close positions is slightly more open than it will be in reality. Thanks to this, in simulation it is easier to move the gripper to the object without detecting a collision. In reality, positions will be slightly more closed to ensure proper grip on objects. 

This functionality can be found in the ERC_2021_simulator repository.

You can download the updated repository or add this functionality to an already existing repository by following the instructions below: 
- in the ERC_2021_simulator repository, in the *simulation* package, create a scripts folder 
- to the created folder, copy the gripper.py file from the ERC_2021_simulator repository in the *simulation* package in the scripts folder 
- update the simulator.launch file, also in the *simulation* package in the launch folder 
- update the robotiq_arg2f_140_model_macro.xacro file located in the *ur_description* package in the urdf folder 

Due to many questions about controlling the manipulator, we provide a tutorial link: https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py. It usesMoveIt! to allow setting joint positions and in the cartesian system. Four minor changes had to be made to work with Univerasl Robots. Changing  the variables: ”group_name” to the name of the group of the robot being used (manipulator). Changing ”boxpose.header.frameid” to ”tool0” and ”grasping_group” to ”manipulator”. It was also necessary to change the number of joints when specifying the position of each joints. The file with the above changes is called moveit_tutorial.py and is located in this repository. 


UPDATE

The simulation includes the UR3 manipulator with an attached gripper (without cover), prepared especially for this year's competition. With MoveIt, you can plan the movement of both the robot and the gripper. An RGB camera was also installed above the gripper, the image of which can be used to detect ArUco tags. The gripper model simulation is based on the robotiq 2f 140 gripper simulation (https://github.com/ros-industrial/robotiq).
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
docker exec -it ur3_simulation_erc /bin/bash
```

Now you can run commands below in docker container.

UR3 simulation in Gazebo with MoveIt! and RViz GUI:
Simulation in Gazebo:
```
roslaunch simulation simulator.launch 
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
git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git
apt install ros-melodic-industrial-core -y
rosdep update
rosdep install --from-paths /catkin_ws/src/ --ignore-src --rosdistro melodic -r -y
sudo apt install ros-melodic-teleop* -y
sudo apt install ros-melodic-joy* -y
sudo apt install ros-melodic-aruco-ros* -y
sudo apt-get install ros-melodic-ros-controllers* -y
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

