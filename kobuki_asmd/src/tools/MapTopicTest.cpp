#include "ros/ros.h"
#include "std_msgs/Int32.h"
//#include "std_msgs/MultiArrayDimension.h"
//#include "std_msgs/MultiArrayLayout.h"


#include <sstream>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "positionTakler");

  ros::NodeHandle mapNode;

  ros::Publisher chatter_pub = mapNode.advertise<std_msgs::Int32>("mapChatter", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  int x = 0;
  int y = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::Int32 pos;
    
    pos.data = x;
    //pos[1].data = y;
    
    ROS_INFO("Position posted [%d]", pos.data);

    chatter_pub.publish(pos);
    
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
    ++x;
    ++y;
  }


  return 0;
}
