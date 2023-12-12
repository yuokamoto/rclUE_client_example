# rclUE_client_example
Example of rclUE client

## Overview
This is client example of rclUE and related UnrealEngine x ROS 2 porject.

Please check following as well
- rclUE documentation

## How to use
### Setup UE project
1. Download Unreal Engine 5 and set up ?? by following README of the repo.
2. Open ?? map and play

### Setup ROS2 ws project
1. Install ROS 2 by following this
2. Create and build ROS 2 ws.
    ```
    mkdir -p colcon_ws/src
    cd colcon_ws/src
    git clone  --recursive git@github.com:yuokamoto/rclUE_client_example.git
    cd ../../
    source /opt/ros/<ros-distro>/setup.bash
    colcon build --packages-select rclUE_client_example
    ```
3. Run example client 
    ```
    source /opt/ros/<ros-distro>/setup.bash
    source colcon_ws/install/setup.bash
    ros2 run rclUE_client_example <example package name>
    ```

## List of examples

*Client script will do following from ROS 2 interface

- Conveyor
    1. Spawn conveyor with mode=1 and speed=200, i.e. payload will move and stop at entrance/exit
    2. Spawn Payload
    3. Wait until payload stop at entrance/exit
    4. Set mode=0, i.e. paylod will move until move out from conveyor
    5. Set speed=-200.
    5. Wait until paylod move out.
    6. Repeat from step 1
- Spline Conveyor
- Elevator
- VerticalConveyor
