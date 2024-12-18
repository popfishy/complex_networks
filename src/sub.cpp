#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ros/console.h>

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Received message: %s", msg->data.c_str());
  setlocale(LC_ALL, "");
  setlocale(LC_CTYPE, "zh_CN.utf8");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;
  
  ros::Subscriber sub = nh.subscribe("chatter", 10, chatterCallback);
  nh.setParam("gain",10);

  ros::spin();

  return 0;
}