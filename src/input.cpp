//Importing necessary libraries
#include <ros/ros.h>
#include <std_msgs/Char.h>

int main(int argc, char **argv)
{
                                                                            //Main function
  ros::init(argc, argv, "user_input");                                      //Initializing ros
  ros::NodeHandle n;                                                        //Creating a nodehandle for the node
  ROS_INFO("Welcome user! \n The following commands are viable: \n While robot not running: \n \n Enter 'q' for quitting the program \n Enter 'r' for resetting the inserted data \n Enter 't' for starting the robot with the inserted data \n \n When the robot is running: \n Enter 'a' for aborting current setup \n"); //Tell the user which options they have

  ros::Publisher user_input = n.advertise<std_msgs::Char>("userinput", 1);  //Creating a publisher for the user_input which publishes to the "userinput"-topic

  while (ros::ok())                                                         //A while loop running only while ros is still running.
  {
    std_msgs::Char begin;                                                   //Declaring a variable of type std_msgs::Char to contain the user_input message
    std::cin >> begin.data;                                                 //Ask the user for input and wait.
    char confirm;                                                           //Declare a variable to contain the user_input confirmation
    if (begin.data == 't')                                                  //If the user input 't'
    {
      ROS_INFO("You sent the command to start the robot");                  //Print a fitting message to the user
      user_input.publish(begin);                                            //Publish the user input to the topic
      confirm = 0;                                                          //Reset the confirmation variable
      begin.data = 0;                                                       //Reset the user_input variable
    }
    else if (begin.data == 'q')                                             //If the user input 'q'
    {
      ROS_WARN("Are you sure you want to terminate? (y/n)");                //Print a message asking the user if they are sure
      std::cin >> confirm;                                                  //Wait for the user to input their answer
      if (confirm == 'y')                                                   //If the user input 'y'
      {
        ROS_WARN("Terminating...");                                         //Print a fitting message to the user
        user_input.publish(begin);                                          //Publish the message to the topic
        ros::Duration(5);                                                   //Wait 5 seconds
        ros::shutdown();                                                    //Shutdown the node
      }
      else                                                                  //If the user inputs anything else but 'y'
      {
        confirm = 0;                                                        //Reset the confirmation variable
        begin.data = 0;                                                     //Reset the user input variable
      }
    }
    else if (begin.data == 'r')                                             //If the user input 'r'
    {
      ROS_WARN("Are you sure you want to reset? (y/n) \n This means that all exhibits previously inserted goals will be deleted"); //Print a message to the user asking if they are sure
      std::cin >> confirm;                                                  //Wait for the user to input their answer
      if (confirm == 'y')                                                   //If the user input 'y'
      {
        user_input.publish(begin);                                          //Publish the message to the topic
        ros::Duration(5);                                                   //Wait 5 seconds
        begin.data = 0;                                                     //Reset the user input variable
        confirm = 0;                                                        //Reset the confirmation variable
      }
      else                                                                  //If the user input anything other than 'y'
      {
        begin.data = 0;                                                     //Reset the user input variable
        confirm = 0;                                                        //Reset the confirmation variable
      }
    }
    else if (begin.data == 'a')                                             //If the user input 'a'
    {
    ROS_INFO("You sent the command to abort current goals");                //Print a fittin message for the user
    user_input.publish(begin);                                              //Publish the message to the topic
    ros::Duration(1);                                                       //Wait 1 second
    begin.data = 0;                                                         //Reset the user_input  
    
    }
    else                                                                    //If the user entered anything other than the above characters
    {
      ROS_WARN("Command not recognised");                                   //Print a message telling them that the command was not recognized
      begin.data = 0;                                                         //Reset the user_input  
    }
  }
  return 0;
}
