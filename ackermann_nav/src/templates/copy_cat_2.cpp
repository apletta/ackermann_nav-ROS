#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ackermann_nav/ControlMsg.h>
#include <iostream>
#include <string>
#include <cstring>

using namespace message_filters;
using namespace std;

void callback(const ackermann_nav::ControlMsg::ConstPtr& talker_info, const ackermann_nav::ControlMsg::ConstPtr& talker_2_info)
{
  //Code goes here

  double talker_store = talker_info->vel_k_in;
  double talker_2_store = talker_2_info->vel_k_in;
  //cout << "test" << " " << talker_store << " " << talker_2_store << endl;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "two_topics_node");

  ros::NodeHandle n;

  message_filters::Subscriber<ackermann_nav::ControlMsg> talker_sub(n, "chatter", 1000);
  message_filters::Subscriber<ackermann_nav::ControlMsg> talker_2_sub(n, "chatter_2", 1000);
  //TimeSynchronizer<ackermann_nav::ControlMsg, ackermann_nav::ControlMsg> sync(talker_sub, talker_2_sub, 10);
  
  typedef sync_policies::ApproximateTime<ackermann_nav::ControlMsg, ackermann_nav::ControlMsg> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), talker_sub, talker_2_sub); //queue size provided by MySyncPolicy(queue_size)
  sync.registerCallback(boost::bind(&callback, _1, _2));


  ros::spin();

  return 0;
}
