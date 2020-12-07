#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <stdlib.h>
#include <time.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf/transform_datatypes.h"
#include <vector>

ros::NodeHandle *ptrnh;
std::vector<move_base_msgs::MoveBaseGoal> targets;
visualization_msgs::MarkerArray markers;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
void userInterface_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);                                                        //Prints messeges containing the received coordinates                                                             //Prints messeges containing the received coordinates
void goal_reached_cb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResult::ConstPtr &result); //Goal has been reached                                                             //Odometry callback function
void send_goal(move_base_msgs::MoveBaseGoal goal_point, int i);                                                                //Send goal to move_base server
void send_marker(move_base_msgs::MoveBaseGoal goal); 
double rob_facing_angle(double angle);
move_base_msgs::MoveBaseGoal get_dif2Dgoal(move_base_msgs::MoveBaseGoal goal);
void sortCoord(std::vector<move_base_msgs::MoveBaseGoal> target, int startpos, int itera, double refx, double refy);
double euclidianDist(double x1, double y1, double refx, double refy);                                                                

int main(int argc, char **argv)
{
  ros::init(argc, argv, "master"); //ros initialisation
  ros::NodeHandle nh2;
  ros::Rate loop(1);
  ptrnh = &nh2;
  ros::Subscriber user_input = nh2.subscribe("new_exhibit", 1, userInterface_cb);    // Subscribes to the user_input topic and when it receives a messege it runs the callback function
  ros::Publisher base_state_pub = nh2.advertise<std_msgs::Bool>("base_state", 5); //Creating a publisher for publishing the state of the MoveBaseClient

  MoveBaseClient ac("move_base", true); //Defining a client to send goals to the move_base server.
  while (!ac.waitForServer(ros::Duration(5.0)))
  {                                                                 //wait for the action server to come up
    ROS_INFO("Waiting for the move_base action server to come up"); //Printing a fitting messege.
  }

  while (ros::ok()) //while(!= ros::Shutdown(); or the user has Ctrl+C out of the program.)
  {
    int i = 0;
    int start = getchar();
    while(start == 't' && i < targets.size())
    {
      if(i == 0)
      {
        sortCoord(targets, i, targets.size(), 0, 0);
        send_goal(targets[i], i);
      }
      else
      {
        sortCoord(targets, i, targets.size(), targets[i-1].target_pose.pose.position.x, targets[i-1].target_pose.pose.position.y);
        send_goal(targets[i], i);
      }
      i++;
    }
    ros::spinOnce();
    if(start == 'q')
    {
      ros::shutdown();
    }
  }
  return 0; //Program run succesfully.
}

void _goal_reached_cb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResult::ConstPtr &result)
{
  if (state.toString() == "SUCCEEDED")
  {
    ROS_INFO("The goal has succesfully been reached!");
    ros::Publisher take_picture = ptrnh->advertise<std_msgs::Bool>("take_picture", 1);
    std_msgs::Bool msg;
    msg.data = true;
    take_picture.publish(msg);
    ros::Rate sleep(0.7);
    sleep.sleep();
    msg.data = false;
    take_picture.publish(msg);
  }
  else
  {
    ROS_INFO("Something went wrong!");
  }
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

  double anglez = rotation.getAngle();
  anglez = rob_facing_angle(anglez);
  goal_target.target_pose.pose.orientation.z = anglez;

  goal_target = get_dif2Dgoal(goal_target);

  rotation.inverse();

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
  ac.sendGoal(goal_point, boost::bind(&_goal_reached_cb, _1, _2));
  ROS_INFO("Sending goal and markers..");
//  send_marker(goal_point);
  ros::Publisher marker_pub = ptrnh->advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
  marker_pub.publish(markers);
  markers.markers.erase(markers.markers.begin()+i);
  markers.markers.shrink_to_fit();
  ac.waitForResult();
}

double rob_facing_angle(double angle)
{
    //This function takes the angle orientation of the particular object and converts it into
    //an angle that would points directly towards the exhibition.

    //beginning of the function

    int oppositeangle = 0;

    if(angle>=0 && angle<=M_PI) //If this is true, the angle of the exhibitions would be added to Pi, to face it with a positive angle
        { 
        oppositeangle = angle+M_PI;
        }
    else if(angle>M_PI && angle<(2*M_PI)) //If this is true, the angle of the exhibitions would be added to Pi, to face it with a positive angle
        { 
        oppositeangle = angle-M_PI;
        } 
    return oppositeangle;
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
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0.7071;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 0.7071;
  marker.lifetime = ros::Duration();
  marker.header.frame_id = goal.target_pose.header.frame_id;
  marker.id = 1;
  marker.pose.position = goal.target_pose.pose.position;
  marker.pose.position.z += marker.scale.x;
  markers.markers.push_back(marker);
  ROS_INFO("Storing marker for publishing"); //For testing purposes.

}

move_base_msgs::MoveBaseGoal get_dif2Dgoal(move_base_msgs::MoveBaseGoal goal)
{
    double dif_x = 0.5*cos(goal.target_pose.pose.orientation.z);
    double dif_y = 0.5*sin(goal.target_pose.pose.orientation.z);

    move_base_msgs::MoveBaseGoal goal_target;
    goal_target.target_pose.pose.orientation.z = goal.target_pose.pose.orientation.z;
    goal_target.target_pose.pose.position.x = goal.target_pose.pose.position.x-dif_x;
    goal_target.target_pose.pose.position.y = goal.target_pose.pose.position.y-dif_y;

    return goal_target;
}

void sortCoord(std::vector<move_base_msgs::MoveBaseGoal> target, int startpos, int itera, double refx, double refy)
{
    //The function takes an array, a starting position, a number of iterations, since it ensures that the array does not get too big,
    //and takes a set of coordinates for the point of reference, It then compares the array's coordinatesets by calling the euclidianDist() function
    //to compare them by their euclidian distance. It then switches the sets if the former set is smaller than the latter.

    //beginning of function

    for(int i = startpos; i<itera; i++) //iterator for the first coordinateset
    {
        if((euclidianDist(target[startpos].target_pose.pose.position.x, target[startpos].target_pose.pose.position.y, refx, refy) > (euclidianDist(target[i].target_pose.pose.position.x, target[i].target_pose.pose.position.y, refx, refy))))
        {
                //switches the places of the coordinateset if it's smaller.
                std::swap(target[startpos], target[i]);
        }
    }
} 

double euclidianDist(double x1, double y1, double refx, double refy)
{
    //This function takes in two coordinates of type double (x1 and y1) and calculates the distance from a point of
    //reference (refx and refy) and returns the euclidian distance between these.

    //beginning of function

    double distx = pow(x1-refx, 2); //Reference distance calculation
    double disty = pow(y1-refy, 2); //Input distance calculation
    double dist = sqrt(distx+disty); //calculation of distance between reference point and input point
    return dist;
}