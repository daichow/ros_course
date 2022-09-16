# Unit Five Notes - ROS Service Servers

## What will you learn with this unit?

* How to give a service
* Create your own service server message

## Creating a Service Server

Run the following commands:

```bash
cd ~/catkin_ws/src

catkin_create_pkg my_service_server_example_pkg rospy std_msgs

cd ~/catkin_ws/src/my_service_server_example_pkg

mkdir scripts

cd scripts

touch simple_service_server.py

chmod +x simple_service_server.py
```
