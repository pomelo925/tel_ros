#ifndef _encoder_H_
#define _encoder_H_

#include "ros/ros.h"
#include "race/microswitch.h"
#include "race/imu.h"
#include <iostream>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>

double p_coe = 1;  // P control coefficient
double x_tol_margin = 1; // x tolerance critical value
double y_tol_margin = 1; // y tolerance critical value
double z_tol_margin = 1; // z tolerance critical value 

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

ros::Publisher encoder_publisher;  // Topic: encoder_toSTM
geometry_msgs::Point encoder_pub;

ros::Subscriber encoder_subscriber;  // Topic: encoder_fromSTM 
geometry_msgs::Point encoder_sub;

// namespace ENCODER is to distinguish it from IMU
namespace ENCODER{
    void init(void);
    void callback(const geometry_msgs::Point::ConstPtr& vel);
    
    void moveTo(double x_cor, double y_cor, double z_cor);
    void moveTo(double x_cor, double y_cor, double z_cor, CH_MICRO condition);

    void moveTo(POINT point);
    void moveTo(POINT point, CH_MICRO condition);
}


#endif