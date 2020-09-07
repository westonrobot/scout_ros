# ROS Packages for Scout Mobile Robot

## Packages

This repository contains minimal packages to control the scout robot using ROS. 

* scout_bringup: launch and configuration files to start ROS nodes 
* scout_base: a ROS wrapper around [wrp_sdk](https://github.com/westonrobot/wrp_sdk) to monitor and control the scout robot
* scout_msgs: scout related message definitions

## Communication interface setup

Please refer to the [README](https://github.com/westonrobot/wrp_sdk#hardware-interface) of "wrp_sdk" package for setup of communication interfaces.

#### Note on CAN interface on Nvidia Jetson Platforms

Nvidia Jeston TX2/Xavier/XavierNX have CAN controller(s) integrated in the main SOC. If you're using a dev kit, you need to add a CAN transceiver for proper CAN communication. 

## Basic usage of the ROS package

1. Install dependent libraries

    ```
    $ sudo apt install -y libasio-dev
    $ sudo apt install -y ros-$ROS_DISTRO-teleop-twist-keyboard
    ```

2. Clone the packages into your catkin workspace and compile

    (the following instructions assume your catkin workspace is at: ~/catkin_ws/src)

    ```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/westonrobot/wrp_sdk.git
    $ git clone https://github.com/westonrobot/scout_base.git
    $ cd ..
    $ catkin_make
    ```

4. Launch ROS nodes
 
* Start the base node for the real robot

    ```
    $ roslaunch scout_bringup scout_minimal.launch
    ```

    The [scout_bringup scout_minimal.launch](scout_bringup/launch/scout_minimal.launch) has 4 parameters:

    - port_name: Determines port to communicate with robot. Default = "can0"
    - simulated_robot: Indicates if launching with a simulation. Default = "false"
    - model_xacro: Indicates the target .xacro file for the publishing of tf frames. Default = [scout_v2.xacro](scout_base/description/scout_v2.xacro)
    - odom_topic_name: Sets the name of the topic which calculated odometry is published. Defaults = "odom"

    or (if you're using a serial port)
        
    ```
    $ roslaunch scout_bringup scout_minimal_uart.launch
    ```

    - Similarly, the [scout_bringup_uart.launch](scout_bringup/launch/scout_minimal_uart.launch) has the same 4 parameters with port_name default = "/dev/ttyUSB0".


* Start the keyboard tele-op node

    ```
    $ roslaunch scout_bringup scout_teleop_keyboard.launch
    ```

    **SAFETY PRECAUSION**: 

    The default command values of the keyboard teleop node are high, make sure you decrease the speed commands before starting to control the robot with your keyboard! Have your remote controller ready to take over the control whenever necessary. 

## Further usage of ROS packages
A brief overview on how to use this ROS package for your custom setup of the scout platform is described in this segment. A detailed example of such applications can be found in the [scout_navigation]() repository.

### Additional Sensors
One possible usage of this package is when additional sensors, such as a lidar is mounted. A new .xacro file will be required to describe the relative position of the new sensor for the publishing of tf frames. 

A [sample](samples/scout_v2_nav.xacro) .xacro file is present in this repository. The base .xacro file of an empty scout platform is included in this sample, and additional links are defined. 

The nodes in this ROS package are made to handle only the control of the scout base and publishing of the status. Additional nodes may need to be created by the user to handle the sensors.

### Alternative Odometry Calculation

Another frequent usage would be using sensor fusion of an IMU. In such a scenario, the odometry calculated by this package would likely be needed to publish under a custom name, instead of "/odom". Therefore, the name of the topic can be set by using

```
$ scout_bringup scout_minimal.launch odom_topic_name:="<custom_name>"
``` 



## Summary of ROS packages

- scout_minimal.launch: Creates ros node that 
