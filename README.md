# ackermann_nav-ROS

## Info
This repository stores the files and setup documents for running the ackermann_nav package in ROS. The ackermann_nav package provides a framework for experimenting with and testing algorithms for four wheeled autonomous vehicles. 

This work was developed for autonomous navigation research conducted at the National University of Singapore (NUS) during Summer 2019. Code is in C++ for a ROS/Gazebo simulation environment. For complete written project report, see AutonomousNavigationBasedOnTurtleBot.pdf

This research project also produced code for a custom MPC controller and ROS testing with TurtleBot. For more information please see [TurtleBot-Navigation](https://github.com/KarthikeyanS27/Turtlebot-Navigation).

## Usage

The intention of this package is to make a user-friendly ROS environment so users can better implement higher level concepts. Documentation is provided explaining the inner workings of the package with the hope that users can easily adjust it to their specific use case. 

All design documents are located in the README folder within the ackermann_nav package. If you want to use this package you should read, at minimum, the "Intro" section of user_guide.pdf.

NOTE: It is strongly recommended to read the design documents before using! This will help you know to what extent this package will work for your application or if you should look elsewhere/make your own package. This should give you an idea of what you can expect from this package out-of-the-box and what is easy/more difficult to adjust. If you already have ROS experience you could jump straight to seeing ackermann_nav_structure.pdf to see how the nodes and topics are set up, though it is still recommended that you read the "Intro" section as well.

## Sample Pics

Using teleop with the model in one of the Gazebo simulation environments (included in this package)
<img src="https://github.com/apletta/ackermann_nav-ROS/blob/master/README/pics/teleop-driving.jpg" alt="teleop driving" width="100%">


The current available simulation worlds are:

1 - Empty

<img src="https://github.com/apletta/ackermann_nav-ROS/blob/master/README/pics/world_empty.png" alt="empty simulation environment" width="60%">

2 - Cone Course

<img src="https://github.com/apletta/ackermann_nav-ROS/blob/master/README/pics/world_cones.png" alt="cone course simulation environment" width="60%">

3 - City

<img src="https://github.com/apletta/ackermann_nav-ROS/blob/master/README/pics/world_city_iso.png" alt="city simulation environment" width="60%">

## Node structure

Block diagram
<img src="https://github.com/apletta/ackermann_nav-ROS/blob/master/README/pics/ackermann_nav_structure.png" alt="block diagram" width="100%"> 

ROS nodes
<img src="https://github.com/apletta/ackermann_nav-ROS/blob/master/README/pics/all_nodes_rqt_graph.png" alt="nodes" width="100%">

Teleop active
<img src="https://github.com/apletta/ackermann_nav-ROS/blob/master/README/pics/teleop.png" alt="teleop active nodes" width="100%">

