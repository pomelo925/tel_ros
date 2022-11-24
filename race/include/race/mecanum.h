#ifndef _MECANUM_H_
#define _MECANUM_H_

#include "ros/ros.h"
#include <iostream>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

// arguments adjustments
const double calibration_x_intercept = 0.7669;
const double calibration_y_intercept = -0.0927;
const double calibration_z_intercept = 0;

const double calibration_x = 6.0207;
const double calibration_y = 6.408;
const double calibration_z = 6.3;
 
const double max_xy = 5.4;
const double max_z = 0.15;
const double acc_xy = 0.03;
const double acc_zz = 0.0008;
 
const double kp = 0.8;
const double fod = 0.25;          // fraction of deceleration: start decelerate at the last dr of the whole distance
const double kp_xy = 1;         // p gain for x- y-direction control
const double kp_z = 0.9;         // p gain for z-direction control 

const double x_tol_margin = 0.5;  // x tolerance critical value
const double y_tol_margin = 0.5;  // y tolerance critical value
const double z_tol_margin = 0.01; // z tolerance critical value
bool data_check;

class POINT{
public:
    double x_cor = 0;
    double y_cor = 0;
    double z_cor = 0;

    POINT(double x, double y){
        this->x_cor = x;
        this->y_cor = y;
    }

    POINT(double x, double y, double z){
        this->x_cor = x;
        this->y_cor = y;
        this->z_cor = z;
    }
};

ros::Publisher mecanum_publisher; // Topic: mecanum_toSTM
geometry_msgs::Point mecanum_pub;

ros::Subscriber mecanum_subscriber; // Topic: mecanum_fromSTM
geometry_msgs::Point mecanum_sub;

// namespace MECANUM is to distinguish it from IMU
namespace MECANUM{
    void init(void);
    void callback(const geometry_msgs::Point::ConstPtr &vel);

    void moveTo(double x_cor, double y_cor, double z_cor);
    void moveTo(POINT point);
    void moveUP(double x_cor, double y_cor, double z_cor);
}

#endif
