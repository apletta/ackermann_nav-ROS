#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <cmath>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
using namespace std;
using std::vector;

/// Declarations

// functions
char getUserInput();
double applyVel(double curr, char input, double step);
void applySteer(double array[3], double curr, char input, double step);

// variables
double steer_control; // value for central steering angle to take
double steer_step; // amount of steering angle change for user left/right input
double left_wheel_angle; // steer angle of left wheel
double right_wheel_angle; // steer angle of right wheel
double wheel_angles[3] = {0.0, 0.0, 0.0}; // array to store left/center/right steer angles
double vel_control; // value for velocity input to rear left/right motors
double vel_step; // amount of velocity change for user forward/back input
char userInput;  // holds user control input

double length; // length between front and rear tires, from center of mass 
double width; // width between left and right tires, from center of mass

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "ackermann_teleop_original");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher left_steer_block_publisher = n.advertise<std_msgs::Float64>("/ackermann/left_steer_block_controller/command", 1000);
  ros::Publisher right_steer_block_publisher = n.advertise<std_msgs::Float64>("/ackermann/right_steer_block_controller/command", 1000);
  ros::Publisher left_rear_wheel_publisher = n.advertise<std_msgs::Float64>("/ackermann/left_rear_wheel_joint_controller/command", 1000);
  ros::Publisher right_rear_wheel_publisher = n.advertise<std_msgs::Float64>("/ackermann/right_rear_wheel_joint_controller/command", 1000);

  ros::Rate loop_rate(10);

  // initialize control commands
  steer_control = 0.0;
  vel_control = 0.0;

  n.param<double>("steer_step", steer_step, 0.1);
  n.param<double>("vel_step", vel_step, 1.0);
  n.param<double>("length", length, 1.2);
  n.param<double>("width", width, 0.8);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
    * This is a message object. You stuff it with data, and then publish it.
    */
    std_msgs::Float64 right_wheel_msg;
    std_msgs::Float64 left_wheel_msg;
    std_msgs::Float64 vel_msg;
    
   // set msg objects
    steer_control = wheel_angles[1];
    left_wheel_angle = wheel_angles[0];
    right_wheel_angle = wheel_angles[2];

    cout << left_wheel_angle << " " << steer_control << " " << right_wheel_angle << endl;
    cout << vel_control << endl;

    left_wheel_msg.data = left_wheel_angle;
    right_wheel_msg.data = right_wheel_angle; 
    vel_msg.data = vel_control;  

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    left_steer_block_publisher.publish(left_wheel_msg);
    right_steer_block_publisher.publish(right_wheel_msg);  
    left_rear_wheel_publisher.publish(vel_msg);
    right_rear_wheel_publisher.publish(vel_msg);

    // node timing
    ros::spinOnce();
    loop_rate.sleep();
    ++count;

    // read in user input (after setting control commands to initial settings)
    userInput = getUserInput();
    
    if(userInput == 's'){
      for(int i=0; i<3; i++){
        wheel_angles[i] = 0.0;
      }
      vel_control = 0.0;
    } else if(userInput=='a' || userInput=='d'){
      applySteer(wheel_angles, steer_control, userInput, steer_step);
    } else if(userInput=='w' || userInput=='x'){
      vel_control = applyVel(vel_control, userInput, vel_step);
    } else if(userInput=='q'){
      break;
    }
  }


  return 0;
}

char getUserInput(){

  cout << "                    " << endl;
  cout << "Control the racecar!" << endl;
  cout << "        w           " << endl;
  cout << "     a  s  d        " << endl;
  cout << "        x           " << endl;
  cout << "                    " << endl;
  cout << "w/x for forward/backward speed" << endl;
  cout << "a/d for turn left/right" << endl;
  cout << "s for e-brake (and lock steering straight)" << endl;
  cout << "                    " << endl;
  cout << "'q' for quit        " << endl;
  char input;
  cin >> input;
  return input;
}

double applyVel(double curr, char input, double step){
  double newVal;
  if(input=='x'){
    newVal = curr+step;
  } else if (input=='w'){
    newVal = curr-step;
  }
  return newVal;
}

void applySteer(double array[3], double curr, char input, double step){
  double newVal;
  if(input=='a'){
    newVal = curr+step;
  } else if (input=='d'){
    newVal = curr-step;
  }

  // check if newVal is effectively zero
  double straight_thresh = 0.00000001;  
  if(newVal>-straight_thresh && newVal <straight_thresh){
    newVal=0.0;
  }
  array[1] = newVal;
  
  double rho; // turning radius from between two front wheels
  rho = length/tan(abs(newVal)); 

  // turning right
  if(newVal<0.0){
    array[0] = -atan(length/(rho+width/2));
    array[2] = -atan(length/(rho-width/2));
  } else if (newVal>0.0){ // turning left
    array[0] = atan(length/(rho-width/2));
    array[2] = atan(length/(rho+width/2));
  } else { // going straight
    array[0] = 0.0;
    array[2] = 0.0;
  }

}
