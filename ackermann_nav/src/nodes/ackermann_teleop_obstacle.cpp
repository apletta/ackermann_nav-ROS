#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/LaserScan.h"

#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <cmath>
#include <string.h>

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
double steer_control = 0.0;
double steer_step = 0.1;
double left_wheel_angle;
double right_wheel_angle;
double wheel_angles[3] = {0.0, 0.0, 0.0};
double vel_control = 0.0;
double vel_step = 1.0;
char userInput;

double length = 1.2;
double width = 0.8;

double range[360];
void checkObstacleFront();

void drive()
{
	
	ros::NodeHandle n;
	
	ros::Publisher left_steer_block_publisher = n.advertise<std_msgs::Float64>("/racecar/left_steer_block_controller/command", 1000);
  ros::Publisher right_steer_block_publisher = n.advertise<std_msgs::Float64>("/racecar/right_steer_block_controller/command", 1000);
  ros::Publisher left_rear_wheel_publisher = n.advertise<std_msgs::Float64>("/racecar/left_rear_wheel_joint_controller/command", 1000);
  ros::Publisher right_rear_wheel_publisher = n.advertise<std_msgs::Float64>("/racecar/right_rear_wheel_joint_controller/command", 1000);
  
  ros::Rate loop_rate(10);

  // initialize control commands
  //steer_control = 0.0;
  //steer_step = 0.1;

  //vel_control = 0.0;
  //vel_step = 1.0;

  //length = 1.2; // wheelbase length, in meters
  //width = 0.8; // wheelbase width, in meters

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  
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
    cout<< left_wheel_angle << " " << steer_control << " " << right_wheel_angle << endl;

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
    checkObstacleFront();
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
      ros::shutdown();
    }
}

void sortLaserRange()
{
  double temp[360];
  for (int i = 0; i < 180; i++)
    temp[i] = range[180+i];
  for (int i = 0; i < 180; i++)
    temp[180+i] = range[i];
  for (int i = 0; i < 360; i++)
  	range[i] = temp[i];
}

void collectRange(const sensor_msgs::LaserScan laser)
{
  for (int i = 0; i < laser.ranges.size(); i++)
  {
     if (laser.ranges[i] == INT_MAX)
     	range[i] = INT_MAX;
     else
     	range[i] = laser.ranges[i];
     //ROS_INFO("range %d: [%f]", i, range[i]);
  }
  sortLaserRange();
  //for (int i = 0; i < 360; i++)
		//cout << "range " << i << ": " << range[i] << "\n";
	drive();
}

void checkObstacleFront()
{
  int startAngle = 85;
  int endAngle = 95;
  double minObstacleDistance = 3.0;
	//for (int i = 0; i < 360; i++)
		//cout << "Distance at " << i << ": " << range[i] << "\n";
  for (int i = 90; i >= startAngle; i--)
  {
  	if (range[i] <= minObstacleDistance)
  		{
  			cout<<"Obstacle Incoming at angle " << i << '\n';
  			userInput='x';
  		}
  }
  for (int i = 90; i <= endAngle; i++)
  {
  	if (range[i] <= minObstacleDistance)
  		{
  			cout<<"Obstacle Incoming at angle " << i << '\n';
  			userInput='x';
  		}
  }
}

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
  ros::init(argc, argv, "talker");
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  //ros::NodeHandle n;
  ros::NodeHandle n1;

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
  /*ros::Publisher left_steer_block_publisher = n.advertise<std_msgs::Float64>("/racecar/left_steer_block_controller/command", 1000);
  ros::Publisher right_steer_block_publisher = n.advertise<std_msgs::Float64>("/racecar/right_steer_block_controller/command", 1000);
  ros::Publisher left_rear_wheel_publisher = n.advertise<std_msgs::Float64>("/racecar/left_rear_wheel_joint_controller/command", 1000);
  ros::Publisher right_rear_wheel_publisher = n.advertise<std_msgs::Float64>("/racecar/right_rear_wheel_joint_controller/command", 1000);*/

  ros::Subscriber lidar_range = n1.subscribe("/racecar/laser_scan", 1000, collectRange);
	ros::spin();
  /*ros::Rate loop_rate(10);

  // initialize control commands
  steer_control = 0.0;
  steer_step = 0.1;

  vel_control = 0.0;
  vel_step = 1.0;

  length = 1.2; // wheelbase length, in meters
  width = 0.8; // wheelbase width, in meters
*/
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  //int count = 0;
  while (ros::ok())
  {
    /**
    * This is a message object. You stuff it with data, and then publish it.
    */
   /* std_msgs::Float64 right_wheel_msg;
    std_msgs::Float64 left_wheel_msg;
    std_msgs::Float64 vel_msg;
    
   // set msg objects
    steer_control = wheel_angles[1];
    left_wheel_angle = wheel_angles[0];
    right_wheel_angle = wheel_angles[2];
    cout<< left_wheel_angle << " " << steer_control << " " << right_wheel_angle << endl;

    left_wheel_msg.data = left_wheel_angle;
    right_wheel_msg.data = right_wheel_angle; 
    vel_msg.data = vel_control;  
*/
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    /*left_steer_block_publisher.publish(left_wheel_msg);
    right_steer_block_publisher.publish(right_wheel_msg);  
    left_rear_wheel_publisher.publish(vel_msg);
    right_rear_wheel_publisher.publish(vel_msg);

    // node timing
    ros::spinOnce();
    loop_rate.sleep();
    ++count;

    // read in user input (after setting control commands to initial settings)
    userInput = getUserInput();
    checkObstacleFront();
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
    }*/
  }


  return 0;
}

char getUserInput(){

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
  }

}
