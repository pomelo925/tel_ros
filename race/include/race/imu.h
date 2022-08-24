#ifndef _IMU_H_
#define _IMU_H_

#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

struct Quaternion {
    double w, x, y, z;
};

struct EulersAngles {
    double roll, pitch, yaw;
};

class CAR_IMU{
private:
    Quaternion Q;
    EulersAngles E;

public:
    void run(void);
    void imu_2_quaternion(double w, double x, double y, double z);
    EulersAngles quaternion_2_euler(void);
    void print_euler(void);
};
extern CAR_IMU Car_imu;

ros::Subscriber imu_subscriber;  // Topic: /imu/data 
void imu_callback(const sensor_msgs::Imu::ConstPtr &ori);

void imu_init(void);
#endif