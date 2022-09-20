# Unit One Notes - ROS Deconstruction

## Differences between roslaunch and rosrun

rosrun can only launch one node at a time, from a single package.

roslaunch can launch two or more nodes at the same time, from multiple packages

## What are Topics

* All communications in ROS happen through the use of topics
* Even services and actions rely on topics
* Topics are named channels for publishing and subscribing to data

## What are Services

* Services are a request/response paradigm
* A client sends a request to a service and waits for a response
* As such, there are service clients and service servers

## What are Actions

* Actions are a request/response paradigm
* Actions are similar to services, but they are parallel processes whereas services are sequential.
* Actions allow for providing feedback while they are being performed