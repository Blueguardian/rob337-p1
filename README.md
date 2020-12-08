# rob337-p1
Project for B337 P1 ROB
Congratiolations user of the Museum scanning robot!
It is currently able to

Changelog:
Initial commit:
Initial part of code, using rviz for sending goals to the code which computes the resulting position of the robot.

Fixed rviz publishing:
Fixed rviz not publishing to the correct topic.

Fixed callback function:
Fixed the msg type the callback function recieves

Spinning:
Added a ros::spin() to the code to loop while waiting for callbacks.
Removed the testing comments from ac.waitforserver();

Fixes:
Added output to the terminal for debugging.

Multiple points algorithm:
Revised rviz config for recieving multiple points before executing.
Added support for multiple points in main.
Revised function for sending points to recieve and store points instead.
Implemented sorting function for sorting for the nearest point.
Revised taking_pictures function.

