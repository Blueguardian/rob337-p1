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
#include <tf2/LinearMath/Quaternion.h>
#include "tf/transform_datatypes.h"
#include <vector>

ros::NodeHandle *ptrnh;                            //Creating a pointer for nodehandle to make it available in functions
std::vector<move_base_msgs::MoveBaseGoal> targets; //A global vector to store the goals in
std::vector<double> angles_recieved;               //A global vector to store the angles from the goals in, for the exhibit scan
int start = 0;                                     //Declaring a variable for user input

//Function prototypes (Prototyping)
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
void userInterface_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);                                                      //Prints messeges containing the received coordinates                                                             //Prints messeges containing the received coordinates
void goal_reached_cb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResult::ConstPtr &result); //Goal has been reached                                                             //Odometry callback function
void send_goal(move_base_msgs::MoveBaseGoal goal_point, int i);                                                              //Send goal to move_base server
void send_marker(move_base_msgs::MoveBaseGoal goal);
void exhib_scan(move_base_msgs::MoveBaseGoal goal, int iter);
void user_input_cb(const std_msgs::Char::ConstPtr &msg);
void get_dif2Dgoal(move_base_msgs::MoveBaseGoal(*goal));
void sortCoord(int startpos, int itera, double refx, double refy);
double rob_facing_angle(double angle);
double euclidianDist(double x1, double y1, double refx, double refy);
double rob_facing_angle(double angle);

int main(int argc, char **argv)                                           //Main function
{
  ros::init(argc, argv, "master");                                        //ros initialisation
  ros::NodeHandle nh2;                                                    //Declaration of a nodehandle
  ros::Rate loop(1);                                                      //Creating a rate at which the program sleeps when told to, here 1 second
  ptrnh = &nh2;                                                           // Assigning the nodehandle pointer to the address of the newly declared nodehandle

  ros::Subscriber user_input = nh2.subscribe("new_exhibit", 1, userInterface_cb); // Subscribes to the user_input topic and when it receives a messege it runs the callback function
  ros::Subscriber input = nh2.subscribe("userinput", 10, user_input_cb);

  MoveBaseClient ac("move_base", true);                                   //Defining a client to send goals to the move_base server, and tell it we want it to keep communication between the program and the server open (For waitforserver)
  while (!ac.waitForServer(ros::Duration(5.0)))
  {                                                                       //wait for the action server to come up
    ROS_INFO("Waiting for the move_base action server to come up");       //Printing a fitting messege.
  }
  int i = 0;                                                              //Iterator for sorting and sending goals
  while (ros::ok())                                                       //while(!= ros::Shutdown(); or the user has not Ctrl+C'ed out of the program.)
  {

    while (start == 't' || start == 'a' && i < targets.size())            //While either t or a has been pressed and iterator i has not reached size of vector targets
    {
      if (i == 0)                                                         //For sending the absolute first goal
      {
        sortCoord(i, targets.size(), 0, 0);                               //Sorting goals for the first goal, with reference point origin [0, 0]
        ROS_INFO("Sending 1. goal");                                      //Printing a fitting message to the user for status.
        send_goal(targets[i], i);                                         //Send the goal to the sending goals function
        loop.sleep();                                                     //Wait 1 sec before continuing
      }
      else                                                                //For sending the rest of the goals
      {
        for (int j = 0; j < targets.size() - i; j++)                      //Running through the rest of the goals regardless of how many there are (Dynamic because of "Unknown amount of goals")
        {
          targets[i].target_pose.header.seq = targets[i].target_pose.header.seq - 1; //Sequencing the goals correctly
        }
        sortCoord(i, targets.size(), targets[i - 1].target_pose.pose.position.x, targets[i - 1].target_pose.pose.position.y); //Sorting the rest of the goals with the reference point being the previous goal.
        ROS_INFO("Sending %d. goal", i + 1);                                                                                  //Sending a fitting messege to the user for status
        send_goal(targets[i], i);                                                                                             //Send the goal to the sending goals function
        loop.sleep();                                                                                                         //Wait 1 sec before continuing
      }
      if (start == 'a')                                                   //If user inputs 'a' during the process
      {
        ROS_INFO("Cancelling goals.. \n Stopping loop..");                //Send the user a messege of what is happenning
        ac.cancelAllGoals();                                              //Tell the robot to stop what it is doing
        ros::Duration(3);                                                 //Wait 3 second before continuing
        start = 0;                                                        //Reset the user_input variable for continuing the code
        ros::Duration(1);                                                 //Wait 1 second before continuing
      }
      i++;                                                                //Increment the iterator
      ros::spinOnce();                                                    //Process all callbacks
    }
    if (start == 'q')                                                     //If the user inputs 'q'
    {
      ROS_INFO("Shutting down..");                                        //Send them a fitting message of what is happening
      ros::shutdown();                                                    //Shut down the program
    }
    else if (start == 'r')                                                //If the user inputs 'r'
    {
      ROS_INFO("Clearing all data..");                                    //Send them a message of what is happening
      angles_recieved.clear();                                            //Clear angles vector to make space for new ones
      angles_recieved.shrink_to_fit();                                    //Reset the size of the vector, to make sure no data is left behind
      targets.clear();                                                    //Clear goals vector to make space for new ones
      targets.shrink_to_fit();                                            //Reset the size of the vector to make sure no data is left behind
      i = 0;                                                              //Reset goal iterator
      loop.sleep();                                                       //Wait 1 second before continuing
      start = 0;                                                          //Reset user_input variable for continuing the code
    }
    ros::spinOnce();                                                      //Process all callbacks
  }
  return 0;                                                               //Program ran succesfully.
}

void _goal_reached_cb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResult::ConstPtr &result)
{
  //Goal reached callback function, not used.

  ros::Duration(1);                                                       // Wait 1 second before continuing
}

void userInterface_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  //Callback function for user_input through the user interface, each time a
  //point has been entered it will be processed and stored for later.

  //Beginning of function

  move_base_msgs::MoveBaseGoal goal_target;                               //Create an object of type move_base_msgs::MoveBaseGoal to store the point in

  goal_target.target_pose.pose.position.x = msg->pose.position.x;         //Assign the x coordinate element in the container the corresponding value in the message
  goal_target.target_pose.pose.position.y = msg->pose.position.y;         //Assign the y coordinate element in the container the corresponding value in the message
  goal_target.target_pose.pose.position.z = msg->pose.position.z;         //Assign the z coordinate element in the container the corresponding value in the message

  tf2::Quaternion rotation;                                               //Create an object of type tf2::Quaternion for a container used for the rotation calculations
  tf2::Vector3 temp_stor;                                                 //Create an object of type tf2::Vector3 for a container used for rotation calculations
  ros::Rate rate(5);                                                      //Create a rate at which the program must sleep before continuing (Here 0.2 seconds) and assign it to an object called rate

  rotation.setX(msg->pose.orientation.x);                                 //Assign the rotation element x in the rotation container to the corresponding value in the message
  rotation.setY(msg->pose.orientation.y);                                 //Assign the rotation element y in the rotation container to the corresponding value in the message
  rotation.setZ(msg->pose.orientation.z);                                 //Assign the rotation element z in the rotation container to the corresponding value in the message
  rotation.setW(msg->pose.orientation.w);                                 //Assign the rotation element w in the rotation container to the corresponding value in the message

  goal_target.target_pose.pose.orientation.z = rotation.getAngle();       //Assign the angle from the message to the goal orientation z element temporarily for calculations
  temp_stor = rotation.getAxis();                                         //Assign the rotational axis the rotation axis from the message for calculations purposes
  get_dif2Dgoal(&goal_target);                                            //Send the current goal to be processed in the first function
  rotation.setRotation(temp_stor, rob_facing_angle(rotation.getAngle())); //Assign the rotation in the container again using the axis and opposite rotation from the function rob_facing_angle

  rate.sleep();                                                           //Wait 0.2 seconds before continuing
  angles_recieved.push_back(rob_facing_angle(rotation.getAngle()));       //Store the resulting angle in the global vector

  goal_target.target_pose.pose.orientation.z = rotation.getZ();           //Assign the final data into the goal object (Rotation)
  goal_target.target_pose.pose.orientation.w = rotation.getW();           //Assign the final data into the goal object (Rotation)
  goal_target.target_pose.pose.orientation.x = rotation.getX();           //Assign the final data into the goal object (Rotation)
  goal_target.target_pose.pose.orientation.y = rotation.getY();           //Assign the final data into the goal object (Rotation)
  rate.sleep();                                                           //Wait 0.2 seconds before continuing
  goal_target.target_pose.header.frame_id = msg->header.frame_id;         //Assign the frame_id to the goal, using the entered point's frame_id
  goal_target.target_pose.header.seq = msg->header.seq + targets.size();  //Assign the correct sequence to the goal object
  rate.sleep();                                                           //Wait 0.2 seconds before continuing
  ROS_INFO("Storing target..");                                           //Print a fitting message for the user
  targets.push_back(goal_target);                                         //Store the target goal in the global vector for goals
}

void send_goal(move_base_msgs::MoveBaseGoal goal_point, int i)
{
  //send_goal is a function for sending goals, which uses an object of type move_base_msgs::MoveBaseGoal and an iterator
  //to send goals and thereafter scan the exhibits for each goal reached.

  //Beginning of function

  MoveBaseClient ac("move_base", true);                                   //Define an action client to send goals to, and tell it that the communication between the program and the robot has to remain open
  ros::Rate rate(5);                                                      //Define a rate at which the program must sleep before continuing assigning it to an object named rate
  ac.sendGoal(goal_point, _goal_reached_cb);                              //Send the goal to the robot, and call the _goal_reached_cb when done
  rate.sleep();                                                           //Wait 0.2 seconds before continuing
  ROS_INFO("Sending goal..");                                             //Print a fitting messege for the user to show the status of the program                                                           
  ac.waitForResult();                                                     //Wait for the robot to finish the current goal
  ROS_INFO("Performing scan of exhibition...");                           //Print a fitting message to the user of the programs status
  rate.sleep();                                                           //Wait 0.2 seconds before continuing
  exhib_scan(goal_point, i);                                              //Call the exhibit scan function before recieving a new goal
}

void get_dif2Dgoal(move_base_msgs::MoveBaseGoal(*goal))
{
  //get_dif2Dgoal is a function for slightly moving the target in relation to the 
  //orientation of the exhibit it moves the point 0.75 meters away from the exhibit
  //in the direction the exhibit is facing

  //Beginning of function

  double dif_x = 0.75 * cos(goal->target_pose.pose.orientation.z);        //Calculate the difference in coordinate x
  double dif_y = 0.75 * sin(goal->target_pose.pose.orientation.z);        //Calculate the difference in coordinate y

  goal->target_pose.pose.orientation.z = goal->target_pose.pose.orientation.z;  //Assign the result to the target goal
  goal->target_pose.pose.position.x = goal->target_pose.pose.position.x + dif_x;//Assign the result in x to the target goal
  goal->target_pose.pose.position.y = goal->target_pose.pose.position.y + dif_y;//Assign the result in y to the target goal
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
  double perp_line_angle = 0;                                             //Calculation of perpendicular angle, used in increment
  double increment_x = 0;                                                 //Declaring a variable for the increments in x and assigning it the value 0;
  double increment_y = 0;                                                 //Declaring a variable for the increments in y and assigning it the value 0;
  bool skip = false;                                                      //Declaring a varible for skipping and assigning the value 'false* to it
  MoveBaseClient ac1("move_base", true);                                  //Define an action client to send goals to, and tell it that the communication between the program and the robot has to remain open

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
  ROS_INFO("Exhibit scanned!..");                                         //Print a fitting message to the user of the status of the program
}

void sortCoord(int startpos, int itera, double refx, double refy)
{
  //sortCoord is a sorting function taking a starting position, and iterator and a reference point in x and y coordinates
  //And sorts a vectors contents for the first element, based on the euclidian distance to each goal.

  //Beginning of function

  for (int i = startpos; i < itera; i++)                                  //Starting a for loop for iterating through the vector
  {
    if ((euclidianDist(targets[startpos].target_pose.pose.position.x, targets[startpos].target_pose.pose.position.y, refx, refy) > (euclidianDist(targets[i].target_pose.pose.position.x, targets[i].target_pose.pose.position.y, refx, refy)))) //Comparing the 2 points based on their euclidian distance to the reference point
    {
      std::swap(targets[startpos], targets[i]);                           //If the goal at the position of the iterator has a smaller euclidian distance to the reference point then swap it with the goal at the startpos
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

  double distx = pow(x1 - refx, 2);                                       //Declaring a variable of type double and assigning it the resulting value
  double disty = pow(y1 - refy, 2);                                       //Declaring a variable of type double and assigning it the resulting value
  double dist = sqrt(distx + disty);                                      //Declaring a variable of type double and assigning it the sum of the earlier varibles
  return dist;                                                            //Returning the last variable as a result
}

void user_input_cb(const std_msgs::Char::ConstPtr &msg)
{
  //The user_input_cb is a callback function for when the user inputs a command for the main program

  //Beginning of function

  ROS_INFO("Command recieved: %d", msg->data); //Print a fitting message for the user.
  start = msg->data; //Assign the info in the message to the global variable for use in the main loop
}

double rob_facing_angle(double angle)
{
  //This function takes the angle orientation of the particular object and converts it into
  //an angle that would points directly towards the exhibition.

  //beginning of the function

  angle = fabs(angle);                                                    //Reassigning the input variable to the absolute of the value it contains
  double oppositeangle = 0;                                               //Declaring a variable of type double and assigning it the value 0;

  if (angle >= 0 && angle <= M_PI)                                        //If this is true, the angle of the exhibitions would be added to Pi, to face it with a positive angle
  {
    oppositeangle = angle + M_PI;                                         //Assigning the oppositeangle value the sum of the input variable and PI
  }
  else if (angle > M_PI && angle < (2 * M_PI))                            //If this is true, the angle of the exhibitions would be added to Pi, to face it with a positive angle
  {
    oppositeangle = angle - M_PI;                                         //Assigning the oppositeangle value the sum of the input variable and -PI
  }
  return oppositeangle;
}
