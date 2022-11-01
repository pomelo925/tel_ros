#ifndef _IMU_H_
#define _IMU_H_

#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Point.h"
#include "race/microswitch.h"
#include "race/mecanum.h"

#define PI 3.14159265358979323846


ros::Subscriber imu_subscriber;  // Topic: /imu/data 
geometry_msgs::Point imu_sub;

ros::Publisher imu_publisher;  // Topic: IMU_Coordinate
geometry_msgs::Point imu_pub;

namespace IMU{
    void init(void);
    void callback(const sensor_msgs::Imu::ConstPtr &ori);
}

class CAR_IMU{
public:
    double roll=0, pitch=0, yaw=0;
    void run(void);
    void get_euler(double x, double y, double z, double w);
    void print_euler(void);
};
extern CAR_IMU Car_imu;

#endif