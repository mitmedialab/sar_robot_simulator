# sar\_robot\_simulator

A ROS package that (in a very simple way) simulates a robot for SAR Year5 Study
game node and translation node testing.

This node will receive RobotCommand messages, sleep to simulate executing
speech and action commands that take time, and publish RobotState messages to
indicate whether it is currently "speaking" or "doing actions".

## Configure and run

From the "sar\_robot\_simulator/src" directory, execute the
robot\_simulator\_node.py file:

`./robot_simulator_node.py`

or 

`python robot_simulator_node.py`

If roscore is not running, the program will print a message saying that it is
unable to register with the master node, and will keep trying to connect.

## ROS messages

The program subscribes to
"/[sar\_robot\_command\_msgs](https://github.com/personal-robots/sar_robot_command_msgs
"sar_robot_command_msgs")/RobotCommand" on the ROS topic
"/robot\_sim\_command".

The program publishes
"/[sar\_robot\_command\_msgs](https://github.com/personal-robots/sar_robot_command_msgs
"sar_robot_command_msgs")/RobotState" on the ROS topic "/robot\_state".

## Version and dependency notes

This program was built and tested with:

- Python 2.7.6 
- ROS Indigo
- sar\_robot\_command\_msgs 1.0.0
- Ubuntu 14.04 LTS (64-bit)

## Bugs and issues

Please report all bugs and issues on the [sar\_robot\_simulator github issues
page](https://github.com/personal-robots/sar_robot_simulator/issues).
