# Unit Four Notes - ROS Subscribers and Messages

## What will you learn with this unit?

* What is a Subscriber and how to create one
* How to create your own message

## What is a ROS Subscriber?

A subscriber is a node that reads information from a topic. Let's execute the next commands:

```bash
cd ~/catkin_ws/src

catkin_create_pkg my_subscriber_example_pkg rospy std_msgs

cd ~/catkin_ws/src/my_subscriber_example_pkg

mkdir scripts

cd ~/catkin_ws/src/my_subscriber_example_pkg/scripts

touch simple_topic_subscriber.py

chmod +x simple_topic_subscriber.py

cd ~/catkin_ws

rm -rf build/ devel/

catkin_make

source devel/setup.bash

rosrun my_subscriber_example_pkg simple_topic_subscriber.py

```

Paste the following code into the **simple_topic_subscriber.py** script:

```python
#! /usr/bin/env python3

import rospy
from std_msgs.msg import Int32


def callback(msg):
    print(msg.data)


rospy.init_node('topic_subscriber')
sub = rospy.Subscriber('/counter', Int32, callback)
rospy.spin()
```

The command `rostopic echo /counter` won't show anything because **nobody is publishing into the /counter topic**. As such, let's publish something to the topic by executing the following commands:

```bash
rostopic pub <topic_name> <message_type> <value>
```

example: `rostopic pub /counter std_msgs/Int32 5`

You should now receive an output of 5 in the rostopic echo terminal.

## Custom Messages Compilation with CMakeLists.txt and package.xml

Now you may be wondering... in case I need to publish some data that is not an Int32, which type of message should I use? You can use all ROS defined (**rosmsg list**) messages. But, in case none fit your needs, you can create a new one.

In order to create a new message, you will need to do the following steps:

1. Create a directory named 'msg' inside your package
2. Inside this directory, create a file named Name_of_your_message.msg (more information down)
3. Modify CMakeLists.txt file (more information down)
4. Modify package.xml file (more information down)
5. Compile
6. Use in code

For example, let's create a message that indicates age, with years, months, and days.

1. Create a directory named 'msg' inside your package

```bash
roscd <package_name>

mkdir msg
```

2. The **Age.msg** file should look like this:

```
float32 years
float32 months
float32 days
```

3, You will have to edit four functions inside CMakeLists.txt:

* find_package()

```cmake
find_package(catkin REQUIRED COMPONENTS
       rospy
       std_msgs
       message_generation   # Add message_generation here, after the other packages
)
```

* add_message_files()

```cmake
add_message_files(
      FILES
      Age.msg
    ) # Dont Forget to UNCOMENT the parenthesis and add_message_files TOO
```

* generate_messages()

```cmake
generate_messages(
      DEPENDENCIES
      std_msgs
) # Dont Forget to uncoment here TOO
```

* catkin_package()

```cmake
catkin_package(
      CATKIN_DEPENDS rospy message_runtime   # This will NOT be the only thing here
)
```

Make your **package.xml** file look like this:

```xml
<?xml version="1.0"?>
<package format="2">
  <name>topic_ex</name>
  <version>0.0.0</version>
  <description>The topic_ex package</description>


  <maintainer email="user@todo.todo">user</maintainer>

  <license>TODO</license>

  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>message_generation</build_depend>
  <build_export_depend>rospy</build_export_depend>
  <exec_depend>rospy</exec_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <exec_depend>std_msgs</exec_depend>
  <build_export_depend>message_runtime</build_export_depend>
  <exec_depend>message_runtime</exec_depend>
  <export>

  </export>
</package>
```

Now, let's compile the package:

```bash
roscd; cd ..

catkin_make

source devel/setup.bash
```

To verify successful message creation, you can execute the following command:

```bash
rosmsg show Age
```