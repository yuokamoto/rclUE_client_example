# rclUE_client_example
Example of rclUE client

## Overview
This is client example of rclUE and related UnrealEngine x ROS 2 porject.

Please check following as well
- [rclUE](https://rclue.readthedocs.io/en/devel/index.html): This repo enables communication between UE and ROS 2.
- [RapyutaSimulationPlugins](https://rapyutasimulationplugins.readthedocs.io/en/devel/index.html): This repo has classes/tools to create ROS 2 enables robots with rclUE.

## How to use
### Setup UE project
1. Download Unreal Engine 5 and set up [rclUE-Examples](https://github.com/yuokamoto/rclUE-Examples/tree/main/Config) by following README of the repo.
2. Open Default map and play

### Setup and run ROS2 ws project
1. Install ROS 2 by following https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html 
2. Create and build ROS 2 ws.
    ```
    mkdir -p colcon_ws/src
    cd colcon_ws/src
    git clone  --recursive git@github.com:yuokamoto/rclUE_client_example.git
    cd ../../
    source /opt/ros/<ros-distro>/setup.bash
    rosdep install --from-paths src -y --ignore-src
    colcon build --symlink-install --packages-up-to rclUE_client_example
    ```
3. Run example client 
    ```
    source /opt/ros/<ros-distro>/setup.bash
    source colcon_ws/install/setup.bash
    ros2 run rclUE_client_example <example package name, e.g. conveyor_client>
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
