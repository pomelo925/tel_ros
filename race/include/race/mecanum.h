#ifndef _MECANUM_H_
#define _MECANUM_H_

#include "ros/ros.h"
#include <iostream>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include "yaml-cpp/yaml.h"

// class POINT{
// public:
//     double x_cor = 0;
//     double y_cor = 0;
//     double z_cor = 0;

//     POINT(double x, double y){
//         this->x_cor = x;
//         this->y_cor = y;
//     }

//     POINT(double x, double y, double z){
//         this->x_cor = x;
//         this->y_cor = y;
//         this->z_cor = z;
//     }
// };

ros::Publisher mecanum_publisher; // Topic: mecanum_toSTM
geometry_msgs::Point mecanum_pub;

ros::Subscriber mecanum_subscriber; // Topic: mecanum_fromSTM
geometry_msgs::Point mecanum_sub;

// namespace MECANUM is to distinguish it from IMU
namespace MECANUM{
    bool data_check;
    void init(void);
    void callback(const geometry_msgs::Point::ConstPtr &vel);

    void moveTo(double x_cor, double y_cor, double z_cor);
    void moveUP(double x_cor, double y_cor, double z_cor);

    void readPath(std::string yaml_path);

/* roslaunch param */
    double calibration_x_intercept = 0.7669;
    double calibration_y_intercept = -0.0927;
    double calibration_z_intercept = 0;
    double calibration_x = 6.0207;
    double calibration_y = 6.408;
    double calibration_z = 6.3;

    double z_overcali_mode = false; 

    double max_xy = 2;
    double min_xy = 0.01;
    double max_z  = 0.15;
    double acc_xy = 0.02;
    double acc_zz = 0.0008; 

    double kp = 0.8;
    double fod_xy = 0.2;     // fraction of deceleration: start decelerate at the last dr of the whole distance
    double fod_z = 0.02;   
    double kp_xy = 0.8;   // p gain for x- y-direction control
    double kp_z = 0.8;    // p gain for z-direction control 

    double x_tol_margin = 0.7;  // x tolerance critical value
    double y_tol_margin = 0.7;  // y tolerance critical value
    double z_tol_margin = 0.001; // z tolerance critical value 
}

#endif
