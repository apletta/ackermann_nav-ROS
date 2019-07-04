//general include statements
#include <ros/ros.h>

//message_filter include statements
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

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
using namespace message_filters;

//class that does all subscribing and publishing
class MultiSubscribeAndPublish
{
public:
  MultiSubscribeAndPublish()
  {
  //set publisher topic and queue size for published messages
  pub_1 = n.advertise<myControlMsg>("published_from_sub3_pub2", 1000);
  pub_2 = n.advertise<myControlMsg>("published_from_sub3_pub2_2", 1000);

  //set subscriber topics and queue size for subscribed messages
  pub1_sub.subscribe(n, "published_from_pub1", 1000);
  pub1_2_sub.subscribe(n, "published_from_pub1_2", 1000);
  pub1_3_sub.subscribe(n, "published_from_pub1_3", 1000);

  //Synchronize topics, args are Sync(MySyncPolicy(<queue size>), <message_field for topic1>, <message_field for topic2>, etc.), queue size determines how soon together messages must be received in order to be synced
  sync.reset(new Sync(MySyncPolicy(10), pub1_sub, pub1_2_sub, pub1_3_sub));
  
  //Make call to callback function, args for _1, _2, etc., must match order and number of topics subscribing to in Sync() call above (ex. first arg after "this" in registerCallback() matches to first arg after "MySyncPolicy() in Sync())
  sync->registerCallback(boost::bind(&MultiSubscribeAndPublish::callback, this, _1, _2, _3));

  }

  //Callback function gets called when receiving messages from ALL subscribed topics within ApproximateTime queue size
  void callback(const myControlMsg::ConstPtr& pub1_info, const myControlMsg::ConstPtr& pub1_2_info, const myControlMsg::ConstPtr& pub1_3_info)
  {
    //create message object(s), need one per topic you want to publish to
    myControlMsg output;
    myControlMsg output2;

    //Read in subscribed info to variables
    double pub1_info_vel = pub1_info->vel_k_in;
    double pub1_info_head = pub1_info->head_k_in;
    double pub1_2_info_vel = pub1_2_info->vel_k_in;
    double pub1_2_info_head = pub1_2_info->head_k_in;
    double pub1_3_info_vel = pub1_3_info->vel_k_in;
    double pub1_3_info_head = pub1_3_info->head_k_in;

    /*


    ALGORITHM CODE GOES HERE


    */


    //fill publish message data fields
    output.header.stamp = ros::Time::now();  //this line likely won't change
    output.vel_k_in = pub1_info_vel;  //this line can be set to your variable
    output.head_k_in = pub1_info_head;  //this line can be set to your variable

    output2.header.stamp = ros::Time::now();  //this line likely won't change
    output2.vel_k_in = pub1_2_info_vel;  //this line can be set to your variable
    output2.head_k_in = pub1_2_info_head;  //this line can be set to your variable
    

    //publish message object(s), could only have one publisher if you want, type to publish must agree with declared publish type of publish object
    pub_1.publish(output);    
    pub_2.publish(output2);
  }

private:

  //create ROS object to access communication operations, only need one
  ros::NodeHandle n;

  //create Publisher objects, one per topic to publish to
  ros::Publisher pub_1;
  ros::Publisher pub_2;

  //Create message_filters so topics can be synchronized, create one per topic to subscribe to
  message_filters::Subscriber<myControlMsg> pub1_sub;
  message_filters::Subscriber<myControlMsg> pub1_2_sub;
  message_filters::Subscriber<myControlMsg> pub1_3_sub;
  

  //define sync policy as ApproximateTime, one type per topic to subscribe to
  typedef sync_policies::ApproximateTime<myControlMsg, myControlMsg, myControlMsg> MySyncPolicy;
  
  //define Synchronizer object with type MySyncPolicy
  typedef Synchronizer<MySyncPolicy> Sync;  //this line likely won't change

  //create variable that will be used to synchronize topics
  boost::shared_ptr<Sync> sync;  //this line likely won't change

};//End of class MultiSubscribeAndPublish

int main(int argc, char** argv)
{
  //Initiate ROS and set node name, should generally be file name
  ros::init(argc, argv, "sub3_pub2");

  //Class object that will subscribe and publish to topics
  MultiSubscribeAndPublish MSAPObject;

  //trigger any callbacks
  ros::spin();

  return 0;
}
