# Behaviortree Plugins

## Overview
This package has to be fulle rebuild in a few weeks when we migrate to jazzy
This project implements behavior tree action nodes for a robot using the `nav2_behavior_tree` framework. The primary functionalities include changing the robot's footprint, checking if a person is in front of the robot, and other behavior tree actions.

## File Structure
```
my_cpp_project
├── include
│   └── nav2_behavior_tree
│       └── plugins
│           └── action
│               ├── change_footprint_action.hpp
│               └── check_person_in_front_action.hpp
├── src
│   ├── change_footprint_action.cpp
│   └── check_person_in_front_action.cpp
├── CMakeLists.txt
└── README.md
```

## Action Nodes

### ChangeFootprintAction
- **Header File**: `include/nav2_behavior_tree/plugins/action/change_footprint_action.hpp`
- **Description**: This class inherits from `BtActionNode` and is responsible for changing the robot's footprint. It includes methods for handling action ticks, waiting for results, and processing success. It provides input ports for specifying the new footprint and the time until the footprint resets.

### CheckPersonInFrontAction
- **Header File**: `include/nav2_behavior_tree/plugins/action/check_person_in_front_action.hpp`
- **Description**: This class also inherits from `BtActionNode` and checks if a person is in front of the robot. It includes methods for action ticks and processing success, and may provide input ports for sensor data or thresholds.


## Usage
After building the project, you can integrate the action nodes into your behavior tree setup. Refer to the documentation of the `nav2_behavior_tree` framework for details on how to use action nodes.
