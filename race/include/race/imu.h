#ifndef _IMU_H_
#define _IMU_H_

#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

double imu_w, imu_x, imu_y, imu_z;
class CAR_IMU{
private:
    double w=0, x=0, y=0, z=0;
    double roll=0, pitch=0, yaw=0;

public:
    void run(void);
    void get_quaternion(void);
    void get_euler(void);
    void print_euler(void);
};
extern CAR_IMU Car_imu;

ros::Subscriber imu_subscriber;  // Topic: /imu/data 
void imu_callback(const sensor_msgs::Imu::ConstPtr &ori);

void imu_init(void);
#endif