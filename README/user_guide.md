# User Guide
### Describes how to use the ackermann_nav package. You should read this, or at least skim, before using so you know how to use and change this package. It is intended to save you time and frustration for later as there are lots of practices to follow so things compile and work together properly. 

1. Launch files: What each does, commands to run them, things to note within the files
2. Package structure: What's in what folders, how files are organized, and generally called by each other
3. Param files: Format, how to use, how to load into a node
4. Messages: Format, how to adjust, how to make your own
5. Topics: Format, how to adjust, how to make your own
6. Nodes: Where to put algorithm code, how to adjust publisher/subscriber topics

## Intro

## Launch files
The "Launch" folder contains all of the package launch files. Launch files allow you to load parameters, run nodes, call other launch files, and more from a single file. You can always run the nodes individually from a single terminal by using the `$ rosrun <package> <node executable name>` command but the launch files make it so you don't need to open say 6 terminals to run 6 nodes.

#### templates_example.launch
To launch the template nodes, run the following command. 

`$ roslaunch ackermann_nav templates_example.launch`

You should see some output for a second or two in the terminal where you executed the launch command. To verify the node structure, open a new terminal and check the rqt_graph.

`$ rosrun rqt_graph rqt_graph`

You should see the following:
![rqt_graph](https://github.com/apletta/ackermann_nav-ROS/blob/master/README/templates_example_rqt_graph.png)


You can also check the nodes and topics using `rostopic list` and `rosnode list`, and additionally get info on each topic and/or node by using `rostopic info <topic>` and `rosnode info <node name>`. Note that topics with no subscribers (i.e. /published_from_sub1_pub1, /published_from_sub3_pub2, /published_from_sub3_pub2_2) don't appear in the rqt_graph. Rqt_graph is generally a good way of visualizing the nodes and connecting topics all at once. 
