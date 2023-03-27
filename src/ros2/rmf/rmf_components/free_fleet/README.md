![](https://github.com/open-rmf/free_fleet/workflows/build/badge.svg)

# Free Fleet

## Contents

- **[About](#About)**
- **[Installation Instructions](#installation-instructions)**
  - [Prerequisites](#prerequisites)
  - [Message Generation](#message-generation)
  - [Client in ROS 1](#client-in-ros-1)
  - [Client and Server in ROS 2](#client-and-server-in-ros-2)
- **[Examples](#examples)**
  - [Barebones Example](#barebones-example)
  - [Turtlebot3 Fleet Server](#turtlebot3-fleet-server)
  - [ROS 1 Turtlebot3 Simulation](#ros-1-turtlebot3-simulation)
  - [ROS 2 Turtlebot3 Simulation](#ros-2-turtlebot3-simulation)
  - [ROS 1 Multi Turtlebot3 Simulation](#ros-1-multi-turtlebot3-simulation)
  - [Commands and Requests](#commands-and-requests)
- **[Plans](#plans)**

</br>
</br>

## About

Welcome to `free_fleet`, an open-source robot fleet management system. 
Sometimes it is called the "Fun Free Fleet For Friends" (F5).

**Note**, this repository is under active development. Things will be quite unstable
for a while. Please open an issue ticket on this repo if you have problems.
Cheers.

</br>
</br>

## Installation Instructions

### Prerequisites

* [Ubuntu 20.04 LTS](https://releases.ubuntu.com/20.04/)
* [ROS1 - Noetic](https://wiki.ros.org/noetic)
* [ROS2 - Galactic](https://docs.ros.org/en/galactic/index.html)

Install all non-ROS prerequisite packages,

```bash
sudo apt update && sudo apt install \
  git wget qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools \
  python3-rosdep \
  python3-vcstool \
  python3-colcon-common-extensions \
  # maven default-jdk   # Uncomment to install dependencies for message generation
```

</br>

### Message Generation

Message generation via `FleetMessages.idl` is done using `dds_idlc` from `CycloneDDS`. For convenience, the generated mesasges and files has been done offline and committed into the code base. They can be found [here](./free_fleet/src/messages/FleetMessages.idl).

```bash
./dds_idlc -allstructs FleetMessages.idl
```

</br>
### Client and Server in ROS 2

Start a new ROS 2 workspace, and pull in the necessary repositories,

```bash
mkdir -p ~/ff_ros2_ws/src
cd ~/ff_ros2_ws/src
git clone https://github.com/open-rmf/free_fleet -b main
git clone https://github.com/open-rmf/rmf_internal_msgs -b main
```

Install all the dependencies through `rosdep`,

```bash
cd ~/ff_ros2_ws
rosdep install --from-paths src --ignore-src --rosdistro galactic -yr
```

Source ROS 2 and build, 

```bash
cd ~/ff_ros2_ws
source /opt/ros/galactic/setup.bash
colcon build

# Optionally use the command below to only build the relevant packages,
# colcon build --packages-up-to \
#     free_fleet ff_examples_ros2 free_fleet_server_ros2 free_fleet_client_ros2
```

</br>
### Commands and Requests

Now the fun begins! There are 3 types of commands/requests that can be sent to the simulated robots through `free_fleet`,

Destination requests, which allows single destination commands for the robots,

```bash
ros2 run ff_examples_ros2 send_destination_request.py -f FLEET_NAME -r ROBOT_NAME -x 1.725 -y -0.39 --yaw 0.0 -i UNIQUE_TASK_ID
```

Path requests, which requests that the robot perform a string of destination commands,

```bash
ros2 run ff_examples_ros2 send_path_request.py -f FLEET_NAME -r ROBOT_NAME -i UNIQUE_TASK_ID -p '[{"x": 1.725, "y": -0.39, "yaw": 0.0, "level_name": "B1"}, {"x": 1.737, "y": 0.951, "yaw": 1.57, "level_name": "B1"}, {"x": -0.616, "y": 1.852, "yaw": 3.14, "level_name": "B1"}, {"x": -0.626, "y": -1.972, "yaw": 4.71, "level_name": "B1"}]'
```

Mode requests which only supports `pause` and `resume` at the moment,

```bash
ros2 run ff_examples_ros2 send_mode_request.py -f FLEET_NAME -r ROBOT_NAME -m pause -i UNIQUE_TASK_ID
ros2 run ff_examples_ros2 send_mode_request.py -f FLEET_NAME -r ROBOT_NAME -m resume -i UNIQUE_TASK_ID
```

**Note** that the task IDs need to be unique, if a request is sent using a previously used task ID, the request will be ignored by the free fleet clients.

</br>
</br>

