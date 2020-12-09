#include <ros/ros.h>
#include <std_msgs/Char.h>

void terminate_cb(const std_msgs::Char::ConstPtr &msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "user_input");
  ros::NodeHandle n;

  ros::Publisher user_input = n.advertise<std_msgs::Char>("userinput", 1);

  while (ros::ok())
  {
    std_msgs::Char begin;
    std::cin >> begin.data;
    if (begin.data == 't')
    {
      ROS_INFO("You sent the command to start the robot");
      user_input.publish(begin);
    }
    else if (begin.data == 'q')
    {
      ROS_WARN("You sent the command to terminate ros");
      user_input.publish(begin);
      ros::Duration(5);
      ros::shutdown();
    }
    else
    {
      ROS_WARN("Command not recognised");
    }
  }
  return 0;
}

void terminate_cb(const std_msgs::Char::ConstPtr &msg)
{
  ros::shutdown();
}