#include "ros/ros.h"
#include <ackermann_nav/ControlMsg.h>

using namespace std;

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<ackermann_nav::ControlMsg>("/published_from_copy_cat", 1000);

    //Topic you want to subscribe
    sub_ = n_.subscribe("/chatter", 1000, &SubscribeAndPublish::callback, this);
  }

  //calback function, takes in message type from subscribed topic
  void callback(const ackermann_nav::ControlMsg& input)
  {
  /////// do something with input here...
  

    cout << "test" << endl;

 
    ackermann_nav::ControlMsg output;  //new message
    //copy data from subscribed data to data that will be published
    output.header.stamp = ros::Time::now();
    output.vel_k_in = input.vel_k_in;
    output.head_k_in = input.vel_k_in;



  /////// publish output here...
    //publish output
    pub_.publish(output);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "subscribe_and_publish");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
