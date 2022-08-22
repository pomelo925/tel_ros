#ifndef _MECANUM_H_
#define _MECANUM_H_

#include "ros/ros.h"
#include "race/microswitch.h"
#include <iostream>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>

double p_coe = 1;  // P control coefficient
double x_tol_margin = 1; // x tolerance critical value
double y_tol_margin = 1; // y tolerance critical value
double z_tol_margin = 1; // z tolerance critical value 

ros::Publisher mecanum_publisher;  // Topic: mecanum_toSTM
geometry_msgs::Point mecanum_pub;

ros::Subscriber mecanum_subscriber;  // Topic: mecanum_fromSTM 
geometry_msgs::Point mecanum_sub;
void mecanum_callback(const geometry_msgs::Point::ConstPtr& vel);

void mecanum_init();
void moveTo(double x_cor, double y_cor);
void moveTo(double x_cor, double y_cor, CH_MICRO condition);
void moveTo(double x_cor, double y_cor, double z_cor);
void moveTo(double x_cor, double y_cor, double z_cor, CH_MICRO condition);



#endif