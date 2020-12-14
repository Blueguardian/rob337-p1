#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <stdlib.h>
#include <time.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Char.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf/transform_datatypes.h"
#include <vector>

ros::NodeHandle *ptrnh; //Creating a pointer for nodehandle to make it available in functions
std::vector<move_base_msgs::MoveBaseGoal> targets; //A global vector to store the goals in
std::vector<double> angles_recieved; //A global vector to store the angles from the goals in, for the exhibit scan
int start = 0; // Variable for user input

//Function prototypes (Prototyping)
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
void userInterface_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);                                                      //Prints messeges containing the received coordinates                                                             //Prints messeges containing the received coordinates
void goal_reached_cb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResult::ConstPtr &result); //Goal has been reached                                                             //Odometry callback function
void send_goal(move_base_msgs::MoveBaseGoal goal_point, int i);                                                              //Send goal to move_base server
void send_marker(move_base_msgs::MoveBaseGoal goal);
void exhib_scan(move_base_msgs::MoveBaseGoal goal, int iter);
void user_input_cb(const std_msgs::Char::ConstPtr &msg);
void get_dif2Dgoal(move_base_msgs::MoveBaseGoal (*goal));
void sortCoord(int startpos, int itera, double refx, double refy);
double rob_facing_angle(double angle);
double euclidianDist(double x1, double y1, double refx, double refy);
double rob_facing_angle(double angle);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "master"); //ros initialisation
  ros::NodeHandle nh2; //Declaration of a nodehandle
  ros::Rate loop(1); //Creating a rate at which the program sleeps when told to, here 1 second
  ptrnh = &nh2; // Assigning the nodehandle pointer to the address of the newly declared nodehandle

  ros::Subscriber user_input = nh2.subscribe("new_exhibit", 1, userInterface_cb); // Subscribes to the user_input topic and when it receives a messege it runs the callback function
  ros::Subscriber input = nh2.subscribe("userinput", 10, user_input_cb);

  MoveBaseClient ac("move_base", true); //Defining a client to send goals to the move_base server.
  while (!ac.waitForServer(ros::Duration(5.0)))
  {                                                                 //wait for the action server to come up
    ROS_INFO("Waiting for the move_base action server to come up"); //Printing a fitting messege.
  }
  int i = 0;
  while (ros::ok()) //while(!= ros::Shutdown(); or the user has Ctrl+C out of the program.)
  {

    while (start == 't' || start == 'a' && i < targets.size())
    {
      if (i == 0)
      {
        sortCoord(i, targets.size(), 0, 0);
        ROS_INFO("Sending 1. goal");
        send_goal(targets[i], i);
        loop.sleep();
      }
      else
      {
        for (int j = 0; j < targets.size() - i; j++)
        {
          targets[i].target_pose.header.seq = targets[i].target_pose.header.seq - 1;
        }
        sortCoord(i, targets.size(), targets[i - 1].target_pose.pose.position.x, targets[i - 1].target_pose.pose.position.y);
        ROS_INFO("Sending %d. goal", i + 1);
        send_goal(targets[i], i);
        loop.sleep();
      }
      if (start == 'a')
      {
        ROS_INFO("Cancelling goals.. \n Stopping loop..");
        ac.cancelAllGoals();
        ros::Duration(3);
        start = 0;
        ros::Duration(1);
      }
      i++;
      ros::spinOnce();
    }
    if (start == 'q')
    {
      ROS_INFO("Shutting down..");
      ros::shutdown();
    }
    else if (start == 'r')
    {
      ROS_INFO("Clearing all data..");
      angles_recieved.clear();
      angles_recieved.shrink_to_fit();
      targets.clear();
      targets.shrink_to_fit();
      i = 0;
      loop.sleep();
      start = 0;
    }
    ros::spinOnce();
  }
  return 0; //Program run succesfully.
}

void _goal_reached_cb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResult::ConstPtr &result)
{
  ros::Duration(1);
}

void userInterface_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  move_base_msgs::MoveBaseGoal goal_target;
  
  goal_target.target_pose.pose.position.x = msg->pose.position.x;
  goal_target.target_pose.pose.position.y = msg->pose.position.y;
  goal_target.target_pose.pose.orientation.z = msg->pose.position.z;

  tf2::Quaternion rotation;
  tf2::Vector3 temp_stor;
  ros::Rate rate(5);

  rotation.setX(msg->pose.orientation.x);
  rotation.setY(msg->pose.orientation.y);
  rotation.setZ(msg->pose.orientation.z);
  rotation.setW(msg->pose.orientation.w);

  goal_target.target_pose.pose.orientation.z = rotation.getAngle();
  temp_stor = rotation.getAxis();
/*
  double dif_x = 0.75 * cos(goal_target.target_pose.pose.orientation.z);
  double dif_y = 0.75 * sin(goal_target.target_pose.pose.orientation.z);
  goal_target.target_pose.pose.position.x = goal_target.target_pose.pose.position.x + dif_x;
  goal_target.target_pose.pose.position.y = goal_target.target_pose.pose.position.y + dif_y;
*/
  get_dif2Dgoal(&goal_target);
  rotation.setRotation(temp_stor, rob_facing_angle(rotation.getAngle()));

  rate.sleep();
  angles_recieved.push_back(rob_facing_angle(rotation.getAngle()));

  goal_target.target_pose.pose.orientation.z = rotation.getZ();
  goal_target.target_pose.pose.orientation.w = rotation.getW();
  goal_target.target_pose.pose.orientation.x = rotation.getX();
  goal_target.target_pose.pose.orientation.y = rotation.getY();
  rate.sleep();
  goal_target.target_pose.header.frame_id = msg->header.frame_id;
  goal_target.target_pose.header.seq = msg->header.seq+targets.size();
  rate.sleep();
  ROS_INFO("Storing target..");
  targets.push_back(goal_target);
}

void send_goal(move_base_msgs::MoveBaseGoal goal_point, int i)
{
  MoveBaseClient ac("move_base", true);
  ros::Rate rate(5);
  ac.sendGoal(goal_point, _goal_reached_cb);
  rate.sleep();
  ROS_INFO("Sending goal..");
  rate.sleep();
  ac.waitForResult();
  ROS_INFO("Performing scan of exhibition...");
  rate.sleep();
  exhib_scan(goal_point, i);
}
void get_dif2Dgoal(move_base_msgs::MoveBaseGoal(*goal))
{
  double dif_x = 0.75 * cos(goal->target_pose.pose.orientation.z);
  double dif_y = 0.75 * sin(goal->target_pose.pose.orientation.z);

  goal->target_pose.pose.orientation.z = goal->target_pose.pose.orientation.z;
  goal->target_pose.pose.position.x = goal->target_pose.pose.position.x + dif_x;
  goal->target_pose.pose.position.y = goal->target_pose.pose.position.y + dif_y;
}

void exhib_scan(move_base_msgs::MoveBaseGoal goal, int iter)
{
  //We save variables for the current exhibit position
  move_base_msgs::MoveBaseGoal tmp_location;
  tmp_location.target_pose.pose.position.x = goal.target_pose.pose.position.x;
  tmp_location.target_pose.pose.position.y = goal.target_pose.pose.position.y;
  tmp_location.target_pose.pose.orientation.z = angles_recieved.at(iter);
  ros::Rate sleep(1);

  //We define how many meters the robot must move each step while re-locating
  double step = 0.15;
  double perp_line_angle = 0; //Calculation of perpendicular angle, used in increment
  double increment_x = 0;
  double increment_y = 0;
  bool skip = false;
  MoveBaseClient ac1("move_base", true);

  if (((fabs(angles_recieved.at(iter)) > 0) && (fabs(angles_recieved.at(iter)) < M_PI_2))) //Angle in fist quadrant
  {                                                                                        //This means the angle can be calculated as
    perp_line_angle = (atan(-1 / (tan(fabs(angles_recieved.at(iter))))));
  }
  else if (((fabs(angles_recieved.at(iter)) > M_PI_2) && (fabs(angles_recieved.at(iter)) < M_PI))) //Second quadrant
  {
    perp_line_angle = (atan(-1 / (tan(M_PI - fabs(angles_recieved.at(iter))))));
  }
  else if (((fabs(angles_recieved.at(iter)) > M_PI) && (fabs(angles_recieved.at(iter)) < (2 * M_PI * (3 / 4))))) //Angle in third quadrant
  {
    perp_line_angle = (atan(-1 / (tan(fabs(angles_recieved.at(iter)) - M_PI))));
  }
  else if ((((fabs(angles_recieved.at(iter)) > (2 * M_PI * (3 / 4)))) && (fabs(angles_recieved.at(iter)) < (2 * M_PI)))) //Angle in fourth quadrant
  {
    perp_line_angle = (atan(-1 / (tan(fabs((fabs(angles_recieved.at(iter)) - (2 * M_PI)))))));
  }

  //We now do calculations which we assign to the datatype goal (x,y and z) in order for the robot to move right/left at the exhibit and take an image.

  if (((fabs(angles_recieved.at(iter)) < 0.01) && (fabs(angles_recieved.at(iter)) > (2 * M_PI - 0.01))) || ((fabs(angles_recieved.at(iter)) < (M_PI + 0.01)) && (fabs(angles_recieved.at(iter)) > (M_PI - 0.01))))
  //The robot has its direction facing 0 or 180 degrees, and only its increments are defined as following:
  {
    increment_x = 0;
    increment_y = step;
    for (int i = 0; i < 3; i++)
    {
      goal.target_pose.pose.position.x = goal.target_pose.pose.position.x + increment_x;
      goal.target_pose.pose.position.y = goal.target_pose.pose.position.y + increment_y;
      ac1.sendGoalAndWait(goal);
      sleep.sleep();
    }
    goal.target_pose.pose.position.x = tmp_location.target_pose.pose.position.x; //Goes back to original position
    goal.target_pose.pose.position.y = tmp_location.target_pose.pose.position.y;

    for (int i = 0; i < 3; i++)
    {
      goal.target_pose.pose.position.x = goal.target_pose.pose.position.x - increment_x;
      goal.target_pose.pose.position.y = goal.target_pose.pose.position.y - increment_y;
      ac1.sendGoalAndWait(goal);
      sleep.sleep();
    }
    skip = true;
  }

  else if (((fabs(angles_recieved.at(iter)) < (M_PI_2 + 0.01)) && (fabs(angles_recieved.at(iter)) > (M_PI_2 - 0.01))) || ((fabs(angles_recieved.at(iter)) < (((3 / 4) * 2 * M_PI) + 0.01)) && (fabs(angles_recieved.at(iter)) > (((3 / 4) * 2 * M_PI) - 0.01))))
  { //The robot has its direction 90 or 270 degrees
    increment_x = step;
    increment_y = 0;
    for (int i = 0; i < 3; i++)
    {
      goal.target_pose.pose.position.x = goal.target_pose.pose.position.x + increment_x;
      goal.target_pose.pose.position.y = goal.target_pose.pose.position.y + increment_y;
      ac1.sendGoalAndWait(goal);
      sleep.sleep();
    }
    goal.target_pose.pose.position.x = tmp_location.target_pose.pose.position.x; //Goes back to original position
    goal.target_pose.pose.position.y = tmp_location.target_pose.pose.position.y;

    for (int i = 0; i < 3; i++)
    {
      goal.target_pose.pose.position.x = goal.target_pose.pose.position.x - increment_x;
      goal.target_pose.pose.position.y = goal.target_pose.pose.position.y - increment_y;
      ac1.sendGoalAndWait(goal);
      sleep.sleep();
    }
    skip = true;
  }
  else //The robot is facing somewhere between - calculations for steps required!
  {
    increment_x = step * cos(fabs(perp_line_angle));
    increment_y = step * sin(fabs(perp_line_angle));
  }

  if (((fabs(angles_recieved.at(iter)) > 0) && (fabs(angles_recieved.at(iter)) < M_PI_2) && (skip == false)) || ((fabs(angles_recieved.at(iter)) > M_PI) && (fabs(angles_recieved.at(iter)) < (2 * M_PI * (3 / 4))) && (skip == false))) //Robot facing either first or third quadrant, as perp_line_angle is positive
  {                                                                                                                                                                                                                                      //The robot has its angle either in first or third quadrant
    for (int i = 0; i < 3; i++)
    { //We make the robot move 3 steps to the right, in which it faces the exhibits
      goal.target_pose.pose.position.x = goal.target_pose.pose.position.x + increment_x;
      goal.target_pose.pose.position.y = goal.target_pose.pose.position.y - increment_y;
      ac1.sendGoalAndWait(goal);
      sleep.sleep();
    }
    goal.target_pose.pose.position.x = tmp_location.target_pose.pose.position.x; //Goes back to original position
    goal.target_pose.pose.position.y = tmp_location.target_pose.pose.position.y;
    for (int i = 0; i < 3; i++)
    { //We make the robot move 3 steps to the right, in which it faces the exhibits
      goal.target_pose.pose.position.x = goal.target_pose.pose.position.x - increment_x;
      goal.target_pose.pose.position.y = goal.target_pose.pose.position.y + increment_y;
      ac1.sendGoalAndWait(goal);
      sleep.sleep();
    }
  }
  else if (((fabs(angles_recieved.at(iter)) > M_PI_2) && (fabs(angles_recieved.at(iter)) < M_PI) && (skip == false)) || ((fabs(angles_recieved.at(iter)) > (2 * M_PI * (3 / 4))) && (fabs(angles_recieved.at(iter)) < (2 * M_PI)) && (skip == false))) //Either second or fourth quadrant, thus negative perp_line_angle
  {
    for (int i = 0; i < 3; i++)
    {
      goal.target_pose.pose.position.x = goal.target_pose.pose.position.x + increment_x;
      goal.target_pose.pose.position.y = goal.target_pose.pose.position.y + increment_y;
      ac1.sendGoalAndWait(goal);
      sleep.sleep();
    }
    goal.target_pose.pose.position.x = tmp_location.target_pose.pose.position.x; //Goes back to original position
    goal.target_pose.pose.position.y = tmp_location.target_pose.pose.position.y;

    for (int i = 0; i < 3; i++)
    {
      goal.target_pose.pose.position.x = goal.target_pose.pose.position.x - increment_x;
      goal.target_pose.pose.position.y = goal.target_pose.pose.position.y - increment_y;
      ac1.sendGoalAndWait(goal);
      sleep.sleep();
    }
  }
  goal.target_pose.pose.position.x = tmp_location.target_pose.pose.position.x; //Goes back to original position
  goal.target_pose.pose.position.y = tmp_location.target_pose.pose.position.y;
  goal.target_pose.pose.orientation.z = tmp_location.target_pose.pose.orientation.z;

  ac1.sendGoalAndWait(goal);
  sleep.sleep();
  ROS_INFO("Exhibit scanned!..");
}

void sortCoord(int startpos, int itera, double refx, double refy)
{
  for (int i = startpos; i < itera; i++)
  {
    if ((euclidianDist(targets[startpos].target_pose.pose.position.x, targets[startpos].target_pose.pose.position.y, refx, refy) > (euclidianDist(targets[i].target_pose.pose.position.x, targets[i].target_pose.pose.position.y, refx, refy))))
    {
      std::swap(targets[startpos], targets[i]);
     /* move_base_msgs::MoveBaseGoal temp;
      temp.target_pose.pose.position.x = targets[startpos].target_pose.pose.position.x;
      temp.target_pose.pose.position.y = targets[startpos].target_pose.pose.position.y;
      temp.target_pose.pose.position.z = targets[startpos].target_pose.pose.position.z;
      temp.target_pose.pose.orientation.x = targets[startpos].target_pose.pose.orientation.x;
      temp.target_pose.pose.orientation.y = targets[startpos].target_pose.pose.orientation.y;
      temp.target_pose.pose.orientation.z = targets[startpos].target_pose.pose.orientation.z;
      temp.target_pose.pose.orientation.w = targets[startpos].target_pose.pose.orientation.w;
      temp.target_pose.header.frame_id = targets[startpos].target_pose.header.frame_id;
      temp.target_pose.header.stamp = targets[startpos].target_pose.header.stamp;
      targets[startpos].target_pose.pose.position.x = targets[i].target_pose.pose.position.x;
      targets[startpos].target_pose.pose.position.y = targets[i].target_pose.pose.position.y;
      targets[startpos].target_pose.pose.position.z = targets[i].target_pose.pose.position.z;
      targets[startpos].target_pose.pose.orientation.x = targets[i].target_pose.pose.orientation.x;
      targets[startpos].target_pose.pose.orientation.y = targets[i].target_pose.pose.orientation.y;
      targets[startpos].target_pose.pose.orientation.z = targets[i].target_pose.pose.orientation.z;
      targets[startpos].target_pose.pose.orientation.w = targets[i].target_pose.pose.orientation.w;
      targets[startpos].target_pose.header.frame_id = targets[i].target_pose.header.frame_id;
      targets[startpos].target_pose.header.stamp = targets[i].target_pose.header.stamp;
      targets[i].target_pose.pose.position.x = temp.target_pose.pose.position.x;
      targets[i].target_pose.pose.position.y = temp.target_pose.pose.position.y;
      targets[i].target_pose.pose.position.z = temp.target_pose.pose.position.z;
      targets[i].target_pose.pose.orientation.x = temp.target_pose.pose.orientation.x;
      targets[i].target_pose.pose.orientation.y = temp.target_pose.pose.orientation.y;
      targets[i].target_pose.pose.orientation.z = temp.target_pose.pose.orientation.z;
      targets[i].target_pose.pose.orientation.w = temp.target_pose.pose.orientation.w;
      targets[i].target_pose.header.frame_id = temp.target_pose.header.frame_id;
      targets[i].target_pose.header.stamp = temp.target_pose.header.stamp; */
    }
  }
}

double euclidianDist(double x1, double y1, double refx, double refy)
{
  //This function takes in two coordinates of type double (x1 and y1) and calculates the distance from a point of
  //reference (refx and refy) and returns the euclidian distance between these.

  //beginning of function

  double distx = pow(x1 - refx, 2);  //Reference distance calculation
  double disty = pow(y1 - refy, 2);  //Input distance calculation
  double dist = sqrt(distx + disty); //calculation of distance between reference point and input point
  return dist;
}

void user_input_cb(const std_msgs::Char::ConstPtr &msg)
{
  ROS_INFO("Command recieved: %d", msg->data);
  start = msg->data;
}

double rob_facing_angle(double angle)
{
  //This function takes the angle orientation of the particular object and converts it into
  //an angle that would points directly towards the exhibition.

  //beginning of the function

  angle = fabs(angle);
  double oppositeangle = 0;

  if (angle >= 0 && angle <= M_PI) //If this is true, the angle of the exhibitions would be added to Pi, to face it with a positive angle
  {
    oppositeangle = angle + M_PI;
  }
  else if (angle > M_PI && angle < (2 * M_PI)) //If this is true, the angle of the exhibitions would be added to Pi, to face it with a positive angle
  {
    oppositeangle = angle - M_PI;
  }
  return oppositeangle;
}
