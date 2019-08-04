//general include statements
#include <ros/ros.h>
#include <std_msgs/Float64.h>

//ackermann_nav message include statements
#include <ackermann_nav/CheckpointMsg.h>
#include <ackermann_nav/ControlMsg.h>
#include <ackermann_nav/GoalMsg.h>
#include <ackermann_nav/ObjectMsg.h>
#include <ackermann_nav/ObjectStateMsg.h>
#include <ackermann_nav/PositionMsg.h>
#include <ackermann_nav/StateMsg.h>
#include <ackermann_nav/TransmissionMsg.h>

//define messages as shorter and easier to read variables, one line per message type to be used by node
typedef ackermann_nav::ControlMsg myControlMsg;

//namespaces
using namespace std;

/* Declarations */
//functions
char getUserInput();

//parameters
double steer_step;
double accel_step;

//variables
double accel_k_in;
double head_k_in;
char userInput;


//main function that runs while node is active, NOT a while loop unless you include one
int main(int argc, char **argv)
{
  //intialize ROS and set node name, should generally be file name
  ros::init(argc, argv, "ackermann_teleop");

  //create ROS object to access communication operations
  ros::NodeHandle n;

  //create Publisher object and set topic to publish to and queue size for published messages
  ros::Publisher pub_1 = n.advertise<myControlMsg>("control_teleop", 1000);

  //set publish rate, in Hz
  ros::Rate loop_rate(10); 


  //initialize parameters
  n.param<double>("steer_step", steer_step, 0.1);
  n.param<double>("accel_step", accel_step, 1.0);


  double count = 0; //counter for running through while loop, used for example
  //while loop that runs while node is active
  while (ros::ok())
  {
    //create message object to get filled with data and then published
    myControlMsg output;
    
    /* CODE GOES HERE */
    userInput = getUserInput(); 

    //reset control variables
    accel_k_in = 0.0;
    head_k_in = 0.0;

   
    if(userInput == 's'){
      accel_k_in = -1000000; //negative infinity is read as code for full brake
      head_k_in = -1000000; //negative infinity is read as code for reset steering
    } else if(userInput=='a'){
        head_k_in = steer_step;
    } else if(userInput=='d'){
        head_k_in = -steer_step;
    } else if(userInput=='w'){
        accel_k_in = accel_step;
    } else if(userInput=='x'){
        accel_k_in = -accel_step;
    } else if(userInput=='q'){
        break;
    } else{
      cout << "Please enter valid input" << endl;
    }
 

    //fill message data fields 
    output.header.stamp = ros::Time::now();  //this line likely won't change
    output.vel_k_in = accel_k_in;  //this line can be set to your variable
    output.head_k_in = head_k_in;  //this line can be set to your variable

    //publish message object, type to publish must agree with declared publish type of publish object
    pub_1.publish(output);

    //trigger any callbacks
    ros::spinOnce();

    //pause as long as needed to meet publish rate
    loop_rate.sleep();

    ++count;  //increment our example counter
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


