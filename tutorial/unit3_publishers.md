# Unit Three Notes - ROS Publishers

## What will you learn with this unit?

* What are ROS topics and how to manage them
* What is a publisher and how to create one
* What are topic messages and how they work

## What is a ROS Topic?

A topic is like a pipe. **Nodes use topics to publish information for other nodes** so that they can communicate.

You can find out, at any time, a list of available topics in a ROS system by doing a `rostopic list`.

You can request information about a topic by doing `rostopic info <name_of_topic>`.

The output indicates the type of information published (std_msgs/Int32), the node that is publishing this information (/topic_publisher), and if there is a node listening to that info (None in this case).

`rostopic echo <name_of_topic>` reads the information that is being published in a topic in realtime.

Example. Create a new package and add rospy and std_msgs as dependencies in the command.

```bash
catkin_create_pkg my_publisher_example_pkg rospy std_msgs

cd my_publisher_example_pkg/

mkdir scripts

cd scripts/

touch simple_topic_publisher.py

chmod +x simple_topic_publisher.py

cd ~/catkin_ws

rm -rf build/ devel/

catkin_make

source devel/setup.bash

rosrun my_publisher_example_pkg simple_topic_publisher.py

```

Paste the following code into the **simple_topics_publisher.py** script:


```python
#! /usr/bin/env python3

# Import the Python library for ROS
import rospy
# Import the Int32 message from the std_msgs package
from std_msgs.msg import Int32             


# Initiate a Node named 'topic_publisher'
rospy.init_node('topic_publisher')

# Create a Publisher object, that will publish on the /counter topic
# messages of type Int32
pub = rospy.Publisher('/counter', Int32, queue_size=1)    
                                           
# Set a publish rate of 2 Hz
rate = rospy.Rate(2)
# Create a variable of type Int32
count = Int32()
# Initialize 'count' variable
count.data = 0                             

# Create a loop that will go until someone stops the program execution
while not rospy.is_shutdown():
  # Publish the message within the 'count' variable
  pub.publish(count)
  # Increment 'count' variable
  count.data += 1
  # Make sure the publish rate maintains at 2 Hz
  rate.sleep()          
  ```

## What are ROS Messages?

Topics handle information through messages. There are many different types of messages.

In the case of the code you executed before, the message type was an **std_msgs/Int32**, but ROS provides a lot of different messages. You can even create your own messages, but it is recommended to use ROS default messages when its possible.

Messages are defined in **.msg** files, which are located inside a **msg** directory of a package.

Get message info with the `rosmsg show <message_type>` command.

Additionally, have a look at the Int32.msg file by running `roscd std_msgs/msg/`.
