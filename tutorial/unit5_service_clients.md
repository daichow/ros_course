# Unit Five Notes - ROS Service Clients

## What will you learn with this unit?

* What a service is
* How to manage services of a robot
* How to call a service

## What's the difference between a Service and an Action?

Services are **Synchronous**. When your ROS program calls a service, your program can't continue until it receives a result from the service.

Actions are **Asynchronous**. It's like launching a new thread. When your ROS program calls an action, your program can perform other tasks while the action is being performed in another thread.

You can list all running services with the command `rosservice list`. Afterwards, get information about a specific service with `rosservice info /<name_of_service>`. The second command outputs two relevant parts of data:

* **Node**: It states the node that provides (has created) that service.

* **Type**: It refers to the kind of message used by this service. It has the same structure as topics do. It's always made of *package_where_the_service_message_is_defined / name_of_the_file_where_service_message_is_defined*.

* **Args**: Here you can find the arguments that this service takes when called. In this case, it only takes a trajectory file path stored in the variable called file.

## How to call a service?

You can call a service manually from the console. This is very useful for testing and having a basic idea of how the service works. 

TAB-TAB means that you have to quickly press the TAB key twice. This will autocomplete the structure of the Service message to be sent for you. Then, you only have to fill in the values.

```bash
rosservice call /the_service_name TAB-TAB
```

## Example

Let's create a new service as an example. Run the following commands:

```bash
cd ~/catkin_ws/src
catkin_create_pkg my_service_client_example_pkg rospy std_msgs
cd ~/catkin_ws/src/my_service_client_example_pkg
mkdir scripts
cd scripts
touch simple_service_client.py
chmod +x simple_service_client.py
```

Now, enter the following code:

```python
#! /usr/bin/env python

import rospy
# Import the service message used by the service /trajectory_by_name
from trajectory_by_name_srv.srv import TrajByName, TrajByNameRequest
import sys

# Initialise a ROS node with the name service_client
rospy.init_node('service_client')
# Wait for the service client /trajectory_by_name to be running
rospy.wait_for_service('/trajectory_by_name')
# Create the connection to the service
traj_by_name_service = rospy.ServiceProxy('/trajectory_by_name', TrajByName)
# Create an object of type TrajByNameRequest
traj_by_name_object = TrajByNameRequest()
# Fill the variable traj_name of this object with the desired value
traj_by_name_object.traj_name = "release_food"
# Send through the connection the name of the trajectory to be executed by the robot
result = traj_by_name_service(traj_by_name_object)
# Print the result given by the service called
print(result)
```
