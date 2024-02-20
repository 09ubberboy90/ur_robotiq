# ur_robotiq

Example of adding a new robots with a different gripper to the telesim_pnp packages suite available [here](https://github.com/cvas-ug/telesim_pnp).

The robot in question is the UR5 with a robotiq gripper. 

This package also publishes a recalculated joint state for the gripper based on the reported opening distance.

This package assumes the `forward_position_controller` from [UR ROS2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) is running. It can be changed from another controller by running `ros control switch_controllers --start forward_position_controller --stop {other_controller}`

## Requirement

- ROS2 (tested on galactic)

## Installation
Clone this repository and build it into your ros workspace using `colcon build`. Make sure to source the workspace before using.

## License
This software is released under MIT liscence.

## Author
Florent Audonnet (https://github.com/09ubberboy90) - Researcher at University of Glasgow, AIST
