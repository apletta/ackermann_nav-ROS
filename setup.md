# Setup
### Describes what you need to do to install and use this package!

1. Make sure your system is running on the following software versions. This package might work with other versions but these are the only ones it has been verified on.
- Linux OS: Ubuntu 18.04 Bionic Beaver
- ROS Distribution: Melodic Morenia
- Gazebo: Version 9 (you can check which version should go with which ROS distribution [here](http://gazebosim.org/tutorials/?tut=ros_wrapper_versions)).

2. Make sure you have a ROS workspace (i.e. catkin_ws) created with at least an `src` folder. 

3. Navigate to that `src` folder and clone this package into it.
- If that for some reason doesn't work:
  
  1. Clone this package into your home directory.
  2. Make a package in `~/catkin_ws/src`.
  
      ex.
      
      `$ cd ~/catkin_ws/src`
     
      `$ catkin_create_pkg ackermann_nav`
  3. Copy all of the files WITHIN the ackermann_nav package into that new package. (Make sure to delete the CMakeLists.txt and package.xml files from the package you just created and copy the ones from your github clone)
  4. Navigate back to `~/catkin_ws` and build everything by running `catkin_make` to make sure it all compiles. 
  
4. Install the [ros_control package](http://wiki.ros.org/ros_control). You will need this so Gazebo can translate ROS messages to motor control. 
     `sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers`
     
     Note: If that command does not work [double check that your install keys are correct](http://answers.ros.org/question/325039/apt-update-fails-cannot-install-pkgs-key-not-working/).
     
5. Have fun and experiment! 
    1. Read the user_guide.md file for launching commands and how to adjust the package for your needs. 
    2. Look through the ackermann_nav_structure.pdf file for package structure. This is important to know so you can write algorithms and/or adjust topics and still interface with the rest of the package!
    3. Refer to the conventions.md file for existing conventions regarding naming and data types.
