# ackermann_nav-ROS

This repository stores the files and setup documents for running the ackermann_nav package in ROS. The ackermann_nav package provides a framework for experimenting with and testing algorithms for four wheeled autonomous vehicles. 

The intention is to make a user-friendly ROS environment so users can better implement higher level concepts. Documentation is provided explaining the inner workings of the package with the hope that users can easily adjust it to their specific use case. 

All design documents are located in the README folder within the ackermann_nav package. If you want to use this package ou should at least read the "Intro" section of the user_guide.pdf.

NOTE: It is strongly recommended to read the design documents before using! This will help you know to what extent this package will work for your application or if you should look elsewhere/make your own package. This should give you an idea of what you can expect from this package out-of-the-box and what is easy/more difficult to adjust. If you already have ROS experience you could jump straight to seeing ackermann_nav_structure.pdf to see how the nodes and topics are set up, though it is still recommended that you read the "Intro" section as well.

Shown here is an example of using teleop with the model in one of the Gazebo simulation environments (included in this package):
<img src="https://github.com/apletta/ackermann_nav-ROS/blob/master/README/pics/teleop-driving.jpg" alt="teleop driving" width="100%">


The current available simulation worlds are:

1 - Empty

<img src="https://github.com/apletta/ackermann_nav-ROS/blob/master/README/pics/world_empty.png" alt="empty simulation environment" width="60%">

2 - Cone Course

<img src="https://github.com/apletta/ackermann_nav-ROS/blob/master/README/pics/world_cones.png" alt="cone course simulation environment" width="60%">

3 - City

<img src="https://github.com/apletta/ackermann_nav-ROS/blob/master/README/pics/world_city_iso.png" alt="city simulation environment" width="60%">
