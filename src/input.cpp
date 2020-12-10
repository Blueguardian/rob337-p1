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
    char confirm;
    if (begin.data == 't')
    {
      ROS_INFO("You sent the command to start the robot");
      user_input.publish(begin);
    }
    else if (begin.data == 'q')
    {
      ROS_WARN("Are you sure you want to terminate? (y/n)");
      confirm = getchar();
      if (confirm == 'y')
      {
        ROS_WARN("Terminating...");
        user_input.publish(begin);
        ros::Duration(5);
        ros::shutdown();
      }
      else
      {
        confirm = 0;
        begin.data = 0;
      }
    }
    else if (begin.data == 'r')
    {
      ROS_WARN("Are you sure you want to reset? \n This means that all exhibits previously inserted goals will be deleted");
      confirm = getchar();
      if (confirm == 'y')
      {
        user_input.publish(begin);
        ros::Duration(5);
      }
      else
      {
        begin.data = 0;
        confirm = 0;
      }
    }
    else
    {
      ROS_WARN("Command not recognised");
    }
  }
  return 0;
}
