# User Guide
### Describes how to use the ackermann_nav package. You should read this, or at least skim, before using so you know how to use and change this package. It is intended to save you time and frustration for later as there are lots of practices to follow so things compile and work together properly. 

## Table of contents
[Intro](#intro) 
- What this package is intended to do and how to get started using it
2. Launch files: What each does, commands to run them, things to note within the files
3. Package structure: What's in what folders, how files are organized, and generally called by each other
4. Param files: Format, how to use, how to load into a node
5. Messages: Format, how to adjust, how to make your own
6. Topics: Format, how to adjust, how to make your own
7. Nodes: Where to put algorithm code, how to adjust publisher/subscriber topics

## Intro

## Launch files
The "Launch" folder contains all of the package launch files. Launch files allow you to load parameters, run nodes, call other launch files, and more from a single file. You can always run the nodes individually from a single terminal by using the `$ rosrun <package> <node executable name>` command but the launch files make it so you don't need to open say 6 terminals to run 6 nodes.

When you execute the launch file, you should see some output for a second or two in the terminal where you executed the launch command. This tells you what the launch file is doing (ex. what parameters are being loaded, what nodes are being launched, etc.).

After launching nodes, it is generally a good idea to confirm that they are running and communicating over topics as you expect them to. You can check the topics and nodes using `rostopic list` and `rosnode list`, and additionally get info on each topic and/or node by using `rostopic info <topic>` and `rosnode info <node name>`. 
- A good way of visualizing everything is look at the rqt_graph by using `rosrun rqt_graph rqt_graph`. Nodes are in boxes, topics in ovals. Note that topics with no subscribers don't appear in the rqt_graph, so for those you'll have to check the topics manually as described above or via some other method. 

### templates_example.launch
To launch the template nodes, run the following command. 

`$ roslaunch ackermann_nav templates_example.launch`

To verify the node structure, open a new terminal and check the rqt_graph.

`$ rosrun rqt_graph rqt_graph`

You should see the following:
![rqt_graph_templates_example](https://github.com/apletta/ackermann_nav-ROS/blob/master/README/pics/templates_example_rqt_graph.png)


### all_nodes.launch
To launch all package nodes, run the following command.
`$ roslaunch ackermann_nav all_nodes.launch`

 To verify the node structure, open a new terminal and check the rqt_graph.

`$ rosrun rqt_graph rqt_graph`

You should see the following:
![rqt_graph_all_nodes](https://github.com/apletta/ackermann_nav-ROS/blob/master/README/pics/all_nodes_rqt_graph.png)


