#ifndef _MECANUM_H_
#define _MECANUM_H_

#include "ros/ros.h"
#include <iostream>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

// arguments adjustments
double kp = 1;
double dr = 0.25;  // start decelerate at the last dr of the whole distance 
double kp_xy = 1.5;  // p gain for x- y-direction control
double kp_z = 0.36;  // p gain for z-direction control
double x_tol_margin = 0.1; // x tolerance critical value
double y_tol_margin = 0.1; // y tolerance critical value
double z_tol_margin = 0.02; // z tolerance critical value 
bool data_check;

class POINT{
public:
    double x_cor=0;
    double y_cor=0;
    double z_cor=0;

    POINT(double x, double y){
        this->x_cor=x; 
        this->y_cor=y;
    }

    POINT(double x, double y, double z){
        this->x_cor=x; 
        this->y_cor=y;
        this->z_cor=z;
    }
};


ros::Publisher mecanum_publisher;  // Topic: mecanum_toSTM
geometry_msgs::Point mecanum_pub;

ros::Subscriber mecanum_subscriber;  // Topic: mecanum_fromSTM 
geometry_msgs::Point mecanum_sub;

ros::Publisher odom_pub;
// tf::TransformBroadcaster odom_broadcaster;  RUNTIME ERROR PROBLEM !!!
geometry_msgs::TransformStamped odom_trans;

// namespace MECANUM is to distinguish it from IMU
namespace MECANUM{
    void init(void);
    void callback(const geometry_msgs::Point::ConstPtr& vel);
    
    void moveTo(double x_cor, double y_cor, double z_cor);
    void moveTo(POINT point);
    void TF(double x, double y, double z, ros::Time time_now);
}

#endif
