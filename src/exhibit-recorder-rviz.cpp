#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>

bool take_picture_check;
ros::NodeHandle *nhptr;

void imageCallback(const sensor_msgs::ImageConstPtr &msg);
void takepic_cb(const std_msgs::Bool::ConstPtr &pic);
sensor_msgs::Image temp_stor;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "exhibit_recorder");
  ros::NodeHandle nh;
  nhptr = &nh;
  ros::Subscriber sub = nh.subscribe("/camera/rgb/image_raw", 1, imageCallback);              //Subscribe to the camera
  ros::Subscriber take_picture = nh.subscribe<std_msgs::Bool>("/takepicture", 1, takepic_cb); //Subscribe to the main function, to know when to take a picture

  while (ros::ok())
  {
  ros::spin();
  }
}
void imageCallback(const sensor_msgs::ImageConstPtr &msg) //Callback function for camera input
{
  temp_stor = *msg;
}

void takepic_cb(const std_msgs::Bool::ConstPtr &pic) //Callback function for taking a picture
{
  bool test = pic.get();
  if (test == true)
  {
    ROS_INFO("Taking picture.."); //Tell the user that the image is being processed on the screen
    take_picture_check = true;    //Tell it to take the picture
    ros::Publisher image_pub = nhptr->advertise<sensor_msgs::Image>("image_view", 1);
    image_pub.publish(temp_stor); 
    ros::Duration(5); //Wait
  }
}