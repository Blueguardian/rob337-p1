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

ros::NodeHandle *ptrnh;
std::vector<move_base_msgs::MoveBaseGoal> targets;
std::vector<double> angles_recieved;
visualization_msgs::MarkerArray markers;
int start = 0;
int id = 0;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
void userInterface_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);                                                      //Prints messeges containing the received coordinates                                                             //Prints messeges containing the received coordinates
void goal_reached_cb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResult::ConstPtr &result); //Goal has been reached                                                             //Odometry callback function
void send_goal(move_base_msgs::MoveBaseGoal goal_point, int i);                                                              //Send goal to move_base server
void send_marker(move_base_msgs::MoveBaseGoal goal);
double rob_facing_angle(double angle);
move_base_msgs::MoveBaseGoal get_dif2Dgoal(move_base_msgs::MoveBaseGoal goal);
void sortCoord(std::vector<move_base_msgs::MoveBaseGoal> target, int startpos, int itera, double refx, double refy);
double euclidianDist(double x1, double y1, double refx, double refy);
void exhib_scan(move_base_msgs::MoveBaseGoal goal, int iter);
void user_input_cb(const std_msgs::Char::ConstPtr &msg);
double rob_facing_angle(double angle);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "master"); //ros initialisation
  ros::NodeHandle nh2;
  ros::Rate loop(1);
  ptrnh = &nh2;

  ros::Subscriber user_input = nh2.subscribe("new_exhibit", 1, userInterface_cb); // Subscribes to the user_input topic and when it receives a messege it runs the callback function
  ros::Publisher base_state_pub = nh2.advertise<std_msgs::Bool>("base_state", 5); //Creating a publisher for publishing the state of the MoveBaseClient
  ros::Subscriber input = nh2.subscribe("userinput", 10, user_input_cb);

  MoveBaseClient ac("move_base", true); //Defining a client to send goals to the move_base server.
  while (!ac.waitForServer(ros::Duration(5.0)))
  {                                                                 //wait for the action server to come up
    ROS_INFO("Waiting for the move_base action server to come up"); //Printing a fitting messege.
  }

  while (ros::ok()) //while(!= ros::Shutdown(); or the user has Ctrl+C out of the program.)
  {
    int i = 0;
    while (start == 't' && i < targets.size())
    {
      if (i == 0)
      {
        ROS_INFO("Sorting targets for closest target...");
        sortCoord(targets, i, targets.size(), 0, 0);
        ROS_INFO("Sending 1. goal");
        send_goal(targets[i], i);
      }
      else
      {
        ROS_INFO("Sorting targets for closest target...");
        sortCoord(targets, i, targets.size(), targets[i - 1].target_pose.pose.position.x, targets[i - 1].target_pose.pose.position.y);
        ROS_INFO("Sending %d. goal", i + 1);
        send_goal(targets[i], i);
      }
      if (start == 'q')
      {
        ROS_INFO("Cancelling goals..");
        ac.cancelAllGoals();
        ROS_INFO("Shutting down..");
        ros::shutdown();
      }
      i++;
      ros::spinOnce();
    }
    if (start == 'q')
    {
      ROS_INFO("Shutting down..");
      ros::shutdown();
    }
    ros::spinOnce();
  }
  return 0; //Program run succesfully.
}

void _goal_reached_cb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResult::ConstPtr &result)
{
  ROS_INFO("The goal has succesfully been reached!");
  ros::Publisher take_picture = ptrnh->advertise<std_msgs::Bool>("/takepicture", 1);
  std_msgs::Bool msg;
  msg.data = true;
  take_picture.publish(msg);
  ros::Rate sleep(0.7);
  sleep.sleep();
  msg.data = false;
  take_picture.publish(msg);
}

void userInterface_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  ROS_INFO("Stored coordinates: [x: %f, y: %f, z: %f]", msg->pose.position.x, msg->pose.position.x, msg->pose.orientation.z); //For testing and errorhandling
  move_base_msgs::MoveBaseGoal goal_target;
  tf2::Quaternion rotation;

  ROS_INFO("Calculating goal position..."); //Testing purposes

  goal_target.target_pose.pose.position.x = msg->pose.position.x;
  goal_target.target_pose.pose.position.y = msg->pose.position.y;
  goal_target.target_pose.pose.orientation.z = msg->pose.position.z;

  rotation.setX(msg->pose.orientation.x);
  rotation.setY(msg->pose.orientation.y);
  rotation.setZ(msg->pose.orientation.z);
  rotation.setW(msg->pose.orientation.w);

  ROS_INFO("Inverting rotation..");

  goal_target.target_pose.pose.orientation.z = rotation.getAngle();
  goal_target = get_dif2Dgoal(goal_target);
  rotation.setRPY(0, 0, rob_facing_angle(rotation.getAngle()));
  angles_recieved.push_back(rob_facing_angle(rotation.getAngle()));
  rotation.normalize();

  goal_target.target_pose.pose.orientation.z = rotation.getZ();
  goal_target.target_pose.pose.orientation.w = rotation.getW();
  goal_target.target_pose.pose.orientation.x = rotation.getX();
  goal_target.target_pose.pose.orientation.y = rotation.getY();
  goal_target.target_pose.header.frame_id = msg->header.frame_id;
  // send_goal(goal_target);
  send_marker(goal_target);
  ROS_INFO("Storing target..");
  targets.push_back(goal_target);
}

void send_goal(move_base_msgs::MoveBaseGoal goal_point, int i)
{
  MoveBaseClient ac("move_base", true);
  ac.sendGoal(goal_point, _goal_reached_cb);
  ROS_INFO("Sending goal and markers..");
  ros::Publisher marker_pub = ptrnh->advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
  marker_pub.publish(markers);
  markers.markers.erase(markers.markers.begin() + i);
  ac.waitForResult();
  ROS_INFO("Performing scan of exhibition...");
  exhib_scan(goal_point, i);
}

void send_marker(move_base_msgs::MoveBaseGoal goal)
{
  //  ros::Publisher marker_pub;
  //  marker_pub = ptrnh->advertise<visualization_msgs::MarkerArray>("exhibit_markers", 1);
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.ns = "target_point";
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 1.0;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.r = 0.1;
  marker.color.g = 0.1;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0.7071;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 0.7071;
  marker.lifetime = ros::Duration();
  marker.header.frame_id = "base link";
  marker.id = id;
  marker.pose.position = goal.target_pose.pose.position;
  marker.pose.position.z += marker.scale.x;
  marker.header.stamp = ros::Time();
  markers.markers.push_back(marker);
  ROS_INFO("Storing marker for publishing"); //For testing purposes.
  id++;
}

move_base_msgs::MoveBaseGoal get_dif2Dgoal(move_base_msgs::MoveBaseGoal goal)
{
  double dif_x = 0.5 * cos(goal.target_pose.pose.orientation.z);
  double dif_y = 0.5 * sin(goal.target_pose.pose.orientation.z);

  move_base_msgs::MoveBaseGoal goal_target;
  goal_target.target_pose.pose.orientation.z = goal.target_pose.pose.orientation.z;
  goal_target.target_pose.pose.position.x = goal.target_pose.pose.position.x - dif_x;
  goal_target.target_pose.pose.position.y = goal.target_pose.pose.position.y - dif_y;

  return goal_target;
}

void exhib_scan(move_base_msgs::MoveBaseGoal goal, int iter)
{
  //We save variables for the current exhibit position
  move_base_msgs::MoveBaseGoal tmp_location;
  tmp_location.target_pose.pose.position.x = goal.target_pose.pose.position.x;
  tmp_location.target_pose.pose.position.y = goal.target_pose.pose.position.y;
  tmp_location.target_pose.pose.orientation.z = angles_recieved.at(iter);

  //We define how many meters the robot must move each step while re-locating
  double step = 0.3;
  double perp_line_angle = 0; //Calculation of perpendicular angle, used in increment
  double increment_x = 0;
  double increment_y = 0;
  bool skip = false;
  MoveBaseClient ac1("move_base", true);

  if (((fabs(angles_recieved.at(iter)) > 0) && (fabs(angles_recieved.at(iter)) < M_PI_2))) //Angle in fist quadrant
  {                                                                                        //This means the angle can be calculated as
    perp_line_angle = fabs(atan(-1 / (tan(fabs(angles_recieved.at(iter))))));
  }
  else if (((fabs(angles_recieved.at(iter)) > M_PI_2) && (fabs(angles_recieved.at(iter)) < M_PI))) //Second quadrant
  {
    perp_line_angle = fabs(atan(-1 / (tan(fabs(angles_recieved.at(iter) - M_PI)))));
  }
  else if (((fabs(angles_recieved.at(iter)) > M_PI) && (fabs(angles_recieved.at(iter)) < (2 * M_PI * (3 / 4))))) //Angle in third quadrant
  {
    perp_line_angle = fabs(atan(-1 / (tan(fabs(angles_recieved.at(iter) - M_PI)))));
  }
  else if ((((fabs(angles_recieved.at(iter)) > (2 * M_PI * (3 / 4)))) && (fabs(angles_recieved.at(iter)) < (2 * M_PI)))) //Angle in fourth quadrant
  {
    perp_line_angle = fabs(atan(-1 / (tan(fabs(angles_recieved.at(iter) - (2 * M_PI))))));
  }

  //We now do calculations which we assign to the datatype goal (x,y and z) in order for the robot to move right/left at the exhibit and take an image.

  if (((fabs(angles_recieved.at(iter)) < 0.01) && (fabs(angles_recieved.at(iter)) > (2 * M_PI - 0.01))) || ((fabs(angles_recieved.at(iter)) < (M_PI + 0.01)) && (fabs(angles_recieved.at(iter)) > (M_PI - 0.01))))
  //The robot has its direction facing 0 or 180 degrees, and only its increments are defines as following:
  {
    increment_x = step;
    increment_y = 0;
    for (int i = 0; i < 3; i++)
    {
      goal.target_pose.pose.position.x = goal.target_pose.pose.position.x + increment_x;
      goal.target_pose.pose.position.y = goal.target_pose.pose.position.y + increment_y;
      ac1.sendGoalAndWait(goal);
    }
    goal.target_pose.pose.position.x = tmp_location.target_pose.pose.position.x; //Goes back to original position
    goal.target_pose.pose.position.y = tmp_location.target_pose.pose.position.y;
    for (int i = 0; i < 3; i++)
    {
      goal.target_pose.pose.position.x = goal.target_pose.pose.position.x - increment_x;
      goal.target_pose.pose.position.y = goal.target_pose.pose.position.y - increment_y;
      ac1.sendGoalAndWait(goal);
    }
    skip = true;
  }

  else if (((fabs(angles_recieved.at(iter)) < (M_PI_2 + 0.01)) && (fabs(angles_recieved.at(iter)) > (M_PI_2 - 0.01))) || ((fabs(angles_recieved.at(iter)) < (((3 / 4) * 2 * M_PI) + 0.01)) && (fabs(angles_recieved.at(iter)) > (((3 / 4) * 2 * M_PI) - 0.01)))) //If cos(angle) = apprx. 1 //6.28 = ca. 2*Pi
  {                                                                                                                                                                                                                                                              //The robot has its direction 90 or 270 degrees
    increment_x = 0;
    increment_y = step;
    for (int i = 0; i < 3; i++)
    {
      goal.target_pose.pose.position.x = goal.target_pose.pose.position.x + increment_x;
      goal.target_pose.pose.position.y = goal.target_pose.pose.position.y + increment_y;
      ac1.sendGoalAndWait(goal);
    }
    goal.target_pose.pose.position.x = tmp_location.target_pose.pose.position.x; //Goes back to original position
    goal.target_pose.pose.position.y = tmp_location.target_pose.pose.position.y;
    for (int i = 0; i < 3; i++)
    {
      goal.target_pose.pose.position.x = goal.target_pose.pose.position.x - increment_x;
      goal.target_pose.pose.position.y = goal.target_pose.pose.position.y - increment_y;
      ac1.sendGoalAndWait(goal);
    }
    skip = true;
  }
  else //The robot is facing somewhere between - calculations for steps required!
  {
    increment_x = step * cos(perp_line_angle);
    increment_y = step * sin(perp_line_angle);
  }

  if (((perp_line_angle > 0) && (perp_line_angle < M_PI_2) && (skip == false)) || (((perp_line_angle > M_PI) && (perp_line_angle < (2 * M_PI * (3 / 4)))) && (skip == false))) //Robot facing either first or third quadrant
  {
    for (int i = 0; i < 3; i++)
    { //We make the robot move 3 steps to the right, in which it faces the exhibits
      goal.target_pose.pose.position.x = goal.target_pose.pose.position.x + increment_x;
      goal.target_pose.pose.position.y = goal.target_pose.pose.position.y - increment_y;
      ac1.sendGoalAndWait(goal);
    }
    goal.target_pose.pose.position.x = tmp_location.target_pose.pose.position.x; //Goes back to original position
    goal.target_pose.pose.position.y = tmp_location.target_pose.pose.position.y;
    for (int i = 0; i < 3; i++)
    { //We make the robot move 3 steps to the right, in which it faces the exhibits
      goal.target_pose.pose.position.x = goal.target_pose.pose.position.x - increment_x;
      goal.target_pose.pose.position.y = goal.target_pose.pose.position.y + increment_y;
      ac1.sendGoalAndWait(goal);
    }
  }
  else if (skip == false)
  {
    for (int i = 0; i < 3; i++)
    {
      goal.target_pose.pose.position.x = goal.target_pose.pose.position.x + increment_x;
      goal.target_pose.pose.position.y = goal.target_pose.pose.position.y + increment_y;
      ac1.sendGoalAndWait(goal);
    }
    goal.target_pose.pose.position.x = tmp_location.target_pose.pose.position.x; //Goes back to original position
    goal.target_pose.pose.position.y = tmp_location.target_pose.pose.position.y;
    for (int i = 0; i < 3; i++)
    {
      goal.target_pose.pose.position.x = goal.target_pose.pose.position.x - increment_x;
      goal.target_pose.pose.position.y = goal.target_pose.pose.position.y - increment_y;
      ac1.sendGoalAndWait(goal);
    }
  }
  goal.target_pose.pose.position.x = tmp_location.target_pose.pose.position.x; //Goes back to original position
  goal.target_pose.pose.position.y = tmp_location.target_pose.pose.position.y;
  ac1.sendGoalAndWait(goal);
}

void sortCoord(std::vector<move_base_msgs::MoveBaseGoal> target, int startpos, int itera, double refx, double refy)
{
  //The function takes an array, a starting position, a number of iterations, since it ensures that the array does not get too big,
  //and takes a set of coordinates for the point of reference, It then compares the array's coordinatesets by calling the euclidianDist() function
  //to compare them by their euclidian distance. It then switches the sets if the former set is smaller than the latter.

  //beginning of function

  for (int i = startpos; i < itera; i++) //iterator for the first coordinateset
  {
    if ((euclidianDist(target[startpos].target_pose.pose.position.x, target[startpos].target_pose.pose.position.y, refx, refy) > (euclidianDist(target[i].target_pose.pose.position.x, target[i].target_pose.pose.position.y, refx, refy))))
    {
      //switches the places of the coordinateset if it's smaller.
      move_base_msgs::MoveBaseGoal temp;
      temp = target[startpos];
      target[startpos] = target[i];
      target[i] = temp;
      //std::swap(target[startpos], target[i]);
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
