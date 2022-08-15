#ifdef _STAGE1_LIB_
#define _STAGE1_LIB_

#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float64.h>

ros::Publisher x_vel_pub;
std_msgs::Float64 x_vel;
ros::Publisher y_vel_pub;
std_msgs::Float64 y_vel;
ros::Publisher rot_pub;
std_msgs::Float64 rot_vel;


void run();
void initial();
void phase1();
void moveTO();

#endif
