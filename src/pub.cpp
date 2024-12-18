#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle nh;
  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 10);
  int gain_ = 5;
  std::string name_="yjq";
  bool init_flag_ = false;

  nh.setParam("talker/gain",gain_);
  nh.setParam("talker/name",name_);
  nh.setParam("talker/init_flag",init_flag_);
  
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    int gain;
    std::string name;
    bool init_flag;
    nh.getParam("talker/gain",gain);
    ROS_INFO("gain is:%d",gain);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}