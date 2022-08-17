#ifndef _VELOCITY_H_
#define _VELOCITY_H_

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <iostream>

double p_coe = 1;  // P control 係數
double x_tol_margin = 1; // x tolerance 的臨界值
double y_tol_margin = 1; // y tolerance 的臨界值
double z_tol_margin = 1; // y tolerance 的臨界值

ros::Publisher vel_publisher;  // topic: vel_toSTM 
geometry_msgs::Point vel_pub;

ros::Subscriber vel_subscriber;  // topic: vel_fromSTM 
geometry_msgs::Point vel_sub;
void vel_callback(const geometry_msgs::Point::ConstPtr& vel);

void velocity_initial();
void moveTo(double x_cor, double y_cor);
void moveTo(double x_cor, double y_cor, double z_cor);
void pos_integral(double x_cor, double y_cor, double x_tolerance, double y_tolerance);

#endif