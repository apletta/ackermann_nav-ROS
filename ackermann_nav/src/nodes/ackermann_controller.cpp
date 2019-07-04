//general include statements
#include <ros/ros.h>

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
typedef ackermann_nav::TransmissionMsg myTransmissionMsg;

//namespaces
using namespace std;



//class that does all subscribing and publishing
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //set publisher topic and queue size for published messages
    pub_1 = n.advertise<myTransmissionMsg>("control_model", 1000);

    //set subscriber topic, queue size for subscribed messages, and callback function to be called whenever messages are detected on topic
    sub_1 = n.subscribe("control_general", 1000, &SubscribeAndPublish::callback, this);
  }


  //callback function, takes in message type from subscribed topic, gets called whenever new messages are detected on subscribed topic
  void callback(const myControlMsg& input)
  {
    //create message object to get filled with data and then published
    myTransmissionMsg output;

    /*



      ALGORITHM CODE GOES HERE



    */


    //fill message data fields
    output.header.stamp = ros::Time::now(); //this line likely won't change
    output.steer_front_l = input.head_k_in; //this line can be set to your variable
    output.steer_front_r = input.head_k_in;  //this line can be set to your variable
    output.vel_rear = input.vel_k_in;     

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
