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
typedef ackermann_nav::TransmissionMsg myTransmissionMsg;

//namespaces
using namespace std;



//class that does all subscribing and publishing
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //set publisher topics and queue sizes for published messages
    left_steer_block_publisher = n.advertise<std_msgs::Float64>("/ackermann/left_steer_block_controller/command", 1000);
    right_steer_block_publisher = n.advertise<std_msgs::Float64>("/ackermann/right_steer_block_controller/command", 1000);
    left_rear_wheel_publisher = n.advertise<std_msgs::Float64>("/ackermann/left_rear_wheel_joint_controller/command", 1000);
    right_rear_wheel_publisher = n.advertise<std_msgs::Float64>("/ackermann/right_rear_wheel_joint_controller/command", 1000);


    //set subscriber topic, queue size for subscribed messages, and callback function to be called whenever messages are detected on topic
    sub_1 = n.subscribe("control_model", 1000, &SubscribeAndPublish::callback, this);
  }


  //callback function, takes in message type from subscribed topic, gets called whenever new messages are detected on subscribed topic
  void callback(const myTransmissionMsg& input)
  {
    
    //read input
    double steer_front_l = input.steer_front_l;
    double steer_front_r = input.steer_front_r;
    double vel_rear = input.vel_rear;

    /* CODE GOES HERE */

    //fill message data fields
    std_msgs::Float64 left_wheel_msg;
    std_msgs::Float64 right_wheel_msg;
    std_msgs::Float64 vel_msg;

    left_wheel_msg.data = steer_front_l;
    right_wheel_msg.data = steer_front_r;
    vel_msg.data = -vel_rear; //using a negative here so that wheels spin properly based on joint coordinate definition

    //publish message object, type to publish must agree with declared publish type of publish object
    left_steer_block_publisher.publish(left_wheel_msg);
    right_steer_block_publisher.publish(right_wheel_msg);
    left_rear_wheel_publisher.publish(vel_msg);
    right_rear_wheel_publisher.publish(vel_msg);

  }

private:
  //create ROS object to access communication operations
  ros::NodeHandle n; 

  //create Publisher object
  ros::Publisher pub_1;
  ros::Publisher left_steer_block_publisher;
  ros::Publisher right_steer_block_publisher;
  ros::Publisher left_rear_wheel_publisher;
  ros::Publisher right_rear_wheel_publisher;

  //create Subscriber object
  ros::Subscriber sub_1;

};//End of class SubscribeAndPublish



//main function that runs while node is active, NOT a while loop unless you include one
int main(int argc, char **argv)
{
  //Initiate ROS and set node name, should generally be file name
  ros::init(argc, argv, "ackermann_gazebo_driver");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  //trigger any callbacks
  ros::spin();

  return 0;
}
