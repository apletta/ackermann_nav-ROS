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
#include <math.h>
#include <cmath>

//define messages as shorter and easier to read variables, one line per message type to be used by node
typedef ackermann_nav::ControlMsg myControlMsg;
typedef ackermann_nav::TransmissionMsg myTransmissionMsg;

//namespaces
using namespace std;
using std::vector;

/* DECLARATIONS */
//functions
double applyVel(double curr, char input);
void applySteer(double array[3], double new_head, double length, double width); 

//variables
double wheel_angles[3] = {0.0, 0.0, 0.0};
double vel_control = 0.0;
double head_control = 0.0;
double dt = 1.0;  
double length;
double width; 


//class that does all subscribing and publishing
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //set publisher topic and queue size for published messages
    pub_1 = n.advertise<myTransmissionMsg>("control_model", 1000);

    //set subscriber topic, queue size for subscribed messages, and callback function to be called whenever messages are detected on topic
    sub_1 = n.subscribe("control_teleop", 1000, &SubscribeAndPublish::callback, this);

    //read in parameters
    n.param<double>("vehicle_length", length, 1.2);
    n.param<double>("vehicle_width", width, 0.8);
  }


  //callback function, takes in message type from subscribed topic, gets called whenever new messages are detected on subscribed topic
  void callback(const myControlMsg& input)
  {
    //create message object to get filled with data and then published
    myTransmissionMsg output;


    /* CODE GOES HERE */

    //read input
    double new_accel = input.vel_k_in;
    double new_head = input.head_k_in;
    
    //apply new control inputs
    //e-brake and realign wheels
    if(new_accel==-1e6 || new_head==-1e6){
      vel_control = 0;
      for(int i=0; i<3; i++){
        wheel_angles[i] = 0;
      }
    } else {
      vel_control = vel_control + new_accel*dt; 
      applySteer(wheel_angles, new_head, length, width);
    }
    
    //print out current wheel angles and speed, useful for testing
    cout << wheel_angles[0] <<  " " << wheel_angles[2] << endl;
    cout << vel_control << endl;


    //fill message data fields
    output.header.stamp = ros::Time::now(); //this line likely won't change
    output.steer_front_l = wheel_angles[0]; //this line can be set to your variable
    output.steer_front_r = wheel_angles[2];  //this line can be set to your variable
    output.vel_rear = vel_control;     

    //publish message object, type to publish must agree with declared publish type of publish object
    pub_1.publish(output);
  }

private:
  //create ROS object to access communication operations
  ros::NodeHandle n; 

  //create Publisher object
  ros::Publisher pub_1;

  //create Subscriber object
  ros::Subscriber sub_1;



};//End of class SubscribeAndPublish



//main function that runs while node is active, NOT a while loop unless you include one
int main(int argc, char **argv)
{
  //Initiate ROS and set node name, should generally be file name
  ros::init(argc, argv, "ackermann_controller");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;
   
  //trigger any callbacks
  ros::spin();

  return 0;
}

void applySteer(double array[3], double new_head, double length, double width){
  double mainHead;
  mainHead = array[1] + new_head;

  // check if newVal is effectively zero
  double straight_thresh = 1e-6;
  if(mainHead>-straight_thresh && mainHead<straight_thresh){
    mainHead=0.0;
  }
  array[1] = mainHead;

  double rho; // turning radius from central axis through front/back of vehicle
  //rho = length/(tan(abs(mainHead))); // bicycle model, mainHead is angle of a wheel in middle of front wheel axis
  rho = length/(2*tan(abs(mainHead))); // point mass in middle of vehicle, mainHead is angle the point will move at

  // turning right
  if(mainHead<0.0){
    array[0] = -atan(length/(rho+width/2));
    array[2] = -atan(length/(rho-width/2));
  } else if (mainHead>0.0){ // turning left
      array[0] = atan(length/(rho-width/2));
      array[2] = atan(length/(rho+width/2));
  } else { // going straight
      array[0] = 0.0;
      array[2] = 0.0;
  }
}

