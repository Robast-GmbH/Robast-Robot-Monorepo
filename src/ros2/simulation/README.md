# Simulation

This directory contains all packages that are only required when running things in the simulation.

## moveit_simulation_agent

This subdirectory contains all resources that are required if you want to test MoveIt applications in the simulation.
This is mainly the translation from ros2 messages to gazebo messages. This can either be done by ros2_control or by
writing a converter (like we did here) that reads out the planed trajectories in ros2 and publishes it to gazebo.

### drawer_bridge_simulation

This package enables the drawers to be opened my utilizing MoveIt.

### gazebo_trajectory_executor

This package contains the code to execute the planned trajectories in gazebo. MoveIt publishes it's planned
trajectories to different ros2 topics / actions depending on what functionality you use.

| MoveIt Component                                    | ros2 interface                                            | default action / topic name                                |
|-----------------------------------------------------|-----------------------------------------------------------|------------------------------------------------------------|
| MoveIt MTC                                          | moveit_task_constructor_msgs::action::ExecuteTaskSolution | /execute_task_solution                                     |
| Planing with move_group_interface with "normal" arm | moveit_msgs::action::ExecuteTrajectory                    | /planning_group_controller/follow_joint_trajectory         |
| Planing with move_group_interface with arm + base   | moveit_msgs::msg::RobotTrajectory                         | /door_opening_mechanism_controller/follow_joint_trajectory |

#### How to run an example?

In order tun ran a MoveIt application in the simulation you have to start the following things:
1. Gazebo (e.g. the task `start tiplu_sim`)
2. Gazebo trajectory executor (depending on what application you run in MoveIt)
   1. For MTC it's e.g. the task `start gazebo_task_solution_executor for door_opening_mechanism`
3. Run MoveIt Application in the MoveIt docker


## oak_d_camera_info_publisher

TODO@Sagar: Fill this with information

## tiplu_world

This package contains a gazebo simulation of the tiplu office.


## Licensing
TODO: fill out licensing information


## Error handle
    simulation: 80
    drawer_bridge_simulation: 81
    gazebo_trajectory_executor: 82
    moveit_simulation: 83
    tiplu_world: 84