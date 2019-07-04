//general include statements
#include <ros/ros.h>
#include <vector>

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
typedef ackermann_nav::GoalMsg myGoalMsg;
typedef ackermann_nav::ObjectStateMsg myObjectStateMsg;
typedef ackermann_nav::PositionMsg myPositionMsg;

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
  pub_1 = n.advertise<myGoalMsg>("goal", 1000);
  pub_2 = n.advertise<myObjectStateMsg>("obstacles_all_known", 1000);

  //set subscriber topics and queue size for subscribed messages
  sub_1.subscribe(n, "obstacles_new", 1000);
  sub_2.subscribe(n, "curr_position", 1000);

  //Synchronize topics, args are Sync(MySyncPolicy(<queue size>), <message_field for topic1>, <message_field for topic2>, etc.), queue size determines how soon together messages must be received in order to be synced
  sync.reset(new Sync(MySyncPolicy(10), sub_1, sub_2));
  
  //Make call to callback function, args for _1, _2, etc., must match order and number of topics subscribing to in Sync() call above (ex. first arg after "this" in registerCallback() matches to first arg after "MySyncPolicy() in Sync())
  sync->registerCallback(boost::bind(&MultiSubscribeAndPublish::callback, this, _1, _2));

  }

  //Callback function gets called when receiving messages from ALL subscribed topics within ApproximateTime queue size
  void callback(const myObjectStateMsg::ConstPtr& sub_1_info, const myPositionMsg::ConstPtr& sub_2_info)
  {
    //create message object(s), need one per topic you want to publish to
    myGoalMsg output;
    myObjectStateMsg output2;

    //Read in subscribed info to variables
    vector<double> sub_1_info_obs_x = sub_1_info->obs_x;
    vector<double> sub_1_info_obs_y = sub_1_info->obs_y;
    double goal_x = 0.0;  //this line can be set to your variable
    double goal_y = 0.0;  //this line can be set to your variable
     
    double sub_2_info_x_k = sub_2_info->x_k;
    double sub_2_info_y_k = sub_2_info->y_k;

    /*


    ALGORITHM CODE GOES HERE


    */


    //fill publish message data fields
    output.header.stamp = ros::Time::now();  //this line likely won't change
    output.goal_x = goal_x;  //this line can be set to your variable
    output.goal_y = goal_y;  //this line can be set to your variable
    output.obs_x = sub_1_info_obs_x;
    output.obs_y = sub_1_info_obs_y;

    output2.header.stamp = ros::Time::now();  //this line likely won't change
    output2.obs_x = sub_1_info_obs_x;  //this line can be set to your variable
    output2.obs_y = sub_1_info_obs_y;  //this line can be set to your variable
    

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
  message_filters::Subscriber<myObjectStateMsg> sub_1;
  message_filters::Subscriber<myPositionMsg> sub_2;
  

  //define sync policy as ApproximateTime, one type per topic to subscribe to
  typedef sync_policies::ApproximateTime<myObjectStateMsg, myPositionMsg> MySyncPolicy;
  
  //define Synchronizer object with type MySyncPolicy
  typedef Synchronizer<MySyncPolicy> Sync;  //this line likely won't change

  //create variable that will be used to synchronize topics
  boost::shared_ptr<Sync> sync;  //this line likely won't change

};//End of class MultiSubscribeAndPublish

int main(int argc, char** argv)
{
  //Initiate ROS and set node name, should generally be file name
  ros::init(argc, argv, "global_map");

  //Class object that will subscribe and publish to topics
  MultiSubscribeAndPublish MSAPObject;

  //trigger any callbacks
  ros::spin();

  return 0;
}
