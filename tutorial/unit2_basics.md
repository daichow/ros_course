# Unit Two Notes - ROS Basics

## What will you learn with this unit?

* How to structure and launch ROS programs (packages and launch files)
* How to create basic ROS programs (Python based)
* Basic ROS concepts: Nodes, Parameter Server, Environment Variables, Roscore.

## What is a ROS package?

ROS uses packages to organize its programs. You can think of a package as **all the files that a specific ROS program contains**;
* cpp files,
* python files,
* configuration files,
* compilation files, 
* launch files, 
* parameters files

All those files in the package are organized with the following structure:

* **launch folder**: Contains launch files
* **src folder**: Source files (cpp, python)
* **CMakeLists.txt**: List of cmake rules for compilation
* **package.xml**: Package information and dependencies

```python
 roscd <package_name>
```
The above command will take you to the package folder.

## Moving robots with ROS

A  ROS program is executed by using some special files called **launch files**.

**roslaunch** is the command used to launch a ROS program. Its structure goes as follows:

```python
roslaunch <package_name> <launch_file>
```

All launch files are contained within a `<launch>` tag. Inside that tag, you can see a `<node>` tag, where we specify the following parameters:

1. **pkg="package_name"** # Name of the package that contains the code of the ROS program to execute
2. **type="python_file_name.py"** # Name of the program file that we want to execute
3. **name="node_name"** # Name of the ROS node that will launch our Python file
4. **output="type_of_output"** # Through which channel you will print the output of the Python file

For example in *turtlebot_teleop.launch/launch/keyboard_teleop.launch*:

```xml
<launch>
  <!-- turtlebot_teleop_key already has its own built in velocity smoother -->
  <node pkg="turtlebot_teleop" type="turtlebot_teleop_key.py" name="turtlebot_teleop_keyboard"  output="screen">
    <param name="scale_linear" value="0.5" type="double"/>
    <param name="scale_angular" value="1.5" type="double"/>
    <remap from="turtlebot_teleop_keyboard/cmd_vel" to="/cmd_vel"/>   <!-- cmd_vel_mux/input/teleop"/-->
  </node>
</launch>
```

## Creating ROS Package

First, goto the **catkin workspace** directory is called **catkin_ws**. Then enter the **catkin_ws/src** directory before typing the following command.

```bash
catkin_create_pkg <package_name> <package_dependecies>
```

In order to test successful creation of the package, type the following commands:

* **rospack list**: Gives you a list with all of the packages in your ROS system.
* **rospack list | grep my_package**: Filters, from all of the packages located in the ROS system, the package named my_package.
* **roscd my_package**: Takes you to the location in the Hard Drive of the package, named my_package.

Now, make a script and save it as **my_script.py** in the **src** folder of the package. Then, make a launch file and save it as **my_launch.launch** in the **launch** folder of the package.

> If you create your Python file from the shell, it may happen that it's created without execution permissions. If this happens, ROS won't be able to find it. If this is your case, you can give execution permissions to the file by typing the next command: **chmod +x name_of_the_file.py**

> Additionally, if roslaunch doesn't work, run the following command **rospack profile** to refresh the ROS package list.

## Creating ROS Nodes

You've already created a ROS node before, but when you type **rosnode list** it won't show up since it is terminated when a python program ends. Use the following code snippet as an example to keep a node running:

```python
#! /usr/bin/env python

import rospy

rospy.init_node("ObiWan")
rate = rospy.Rate(2)               # We create a Rate object of 2Hz
while not rospy.is_shutdown():     # Endless loop until Ctrl + C
   print("Help me Obi-Wan Kenobi, you're my only hope")
   rate.sleep()                    # We sleep the needed time to maintain the Rate fixed above
    
# This program creates an endless loop that repeats itself 2 times per second (2Hz) until somebody presses Ctrl + C
# in the Shell
```

Enter **rosnode info <node_name>** to get information about a node.

## Compiling ROS Packages

In order to compile a ROS package, you need to be in the **catkin_ws** directory. Then, type the following command:

```bash
cd ~/catkin_ws

catkin_make

source devel/setup.bash
```
Make sure to source the workspace so that ROS will always get the latest changes done in your workspace.

## ROS Parameter Server

A Parameter Server is a **dictionary** that ROS uses to store parameters. These parameters can be used by nodes at runtime and are normally used for static data, such as configuration parameters.

These are the commands used to interact with the Parameter Server:

```bash
rosparam list
rosparam get <parameter_name>
rosparam set <parameter_name> <value>
```

## Roscore

The roscore is the main process that manages all of the ROS system. You always need to have a roscore running in order to work with ROS.

![roscore](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_ROS/img/roscore.jpg)

## Environment Variables

ROS uses a set of Linux system environment variables in order to work properly. You can check these variables by typing:
  
```bash
export | grep ROS
```

output:

```bash
declare -x ROSLISP_PACKAGE_DIRECTORIES="/home/user/catkin_ws/devel/share/common-lisp"
declare -x ROS_DISTRO="noetic"
declare -x ROS_ETC_DIR="/opt/ros/noetic/etc/ros"
declare -x ROS_MASTER_URI="http://localhost:11311"
declare -x ROS_PACKAGE_PATH="/home/user/catkin_ws/src:/opt/ros/noetic/share:/opt/ros/noetic/stacks"
declare -x ROS_ROOT="/opt/ros/noetic/share/ros"
```

The following are the most important environment variables:

```bash
ROS_MASTER_URI -> Contains the url where the ROS Core is being executed. Usually, your own computer (localhost).
ROS_PACKAGE_PATH -> Contains the paths in your Hard Drive where ROS has packages in it.
```