#include <ros/ros.h>
#include <std_msgs/Char.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "user_input");
  ros::NodeHandle n;

  ros::Publisher user_input =n.advertise<std_msgs::Char>("user_input", 1);

  while (ros::ok())
  {
    std_msgs::Char begin;
    std::cin >> begin.data;
    user_input.publish(begin);
  }
  return 0;
}