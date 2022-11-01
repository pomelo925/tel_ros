#include "race/imu.h"
#include <cmath>

CAR_IMU Car_imu;

void IMU::init(void){
    ros::NodeHandle nh_4imu;
    imu_subscriber = nh_4imu.subscribe("/imu/data", 1, IMU::callback);
    imu_publisher = nh_4imu.advertise<geometry_msgs::Point>("IMU_info", 1);
}

void IMU::callback(const sensor_msgs::Imu::ConstPtr &ori){
    Car_imu.get_euler(ori->orientation.w, ori->orientation.x, ori->orientation.y, ori->orientation.z);
    Car_imu.print_euler();
}

void CAR_IMU::get_euler(double x, double y, double z, double w){
    double len = sqrt(w*w + x*x + y*y + z*z);
    w /= len;
    x /= len;
    y /= len;
    z /= len;

    // roll (x-axis rotation)
    double q32 = 2 * (y*z + w*x);
    double q33 = 1 - 2 * (x*x + y*y);
    roll = atan(q32/q33) * 180/PI;

    // pitch (y-axis rotation)
    double q31 = 2 * (x*z - w*y);
    pitch = asin(-q31) * 180/PI;

    // yaw (z-axis rotation)
    double q21 = 2 * (x*y - w*z);
    double q11 = 1 - 2 * (y*y + z*z);
    yaw = atan(q21/q11) * 180/PI;

    imu_pub.x = roll;
    imu_pub.y = pitch;
    imu_pub.z = yaw;
    imu_publisher.publish(imu_pub); 
}

void CAR_IMU::print_euler(void){
    ROS_INFO("=== Euler Coordinate (unit: degree)===\n");
    ROS_INFO("PITCH (X axis): %lf\n", this->pitch);
    ROS_INFO("ROLL  (Y axis): %lf\n", this->roll);
    ROS_INFO("YAW   (Z axis): %lf\n", this->yaw);
}



