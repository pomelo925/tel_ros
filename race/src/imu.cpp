#include "race/imu.h"
#include <cmath>

CAR_IMU Car_imu;

void imu_init(void){
    ros::NodeHandle nh_4imu;
    imu_subscriber = nh_4imu.subscribe("/imu/data", 1, imu_callback);
    imu_publisher = nh_4imu.advertise<geometry_msgs::Point>("IMU_info", 1);
}

void CAR_IMU::get_quaternion(void){
    this->w = imu_w;
    this->x = imu_x;
    this->y = imu_y;
    this->z = imu_z;
}

void CAR_IMU::get_euler(void){
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
    ROS_INFO("=== Quaternion Coordinate ===\n");
    ROS_INFO("w : %lf\n", imu_w);
    ROS_INFO("x : %lf\n", imu_x);
    ROS_INFO("y : %lf\n", imu_y);
    ROS_INFO("z : %lf\n", imu_z);

    ROS_INFO("=== Euler Coordinate (unit: degree)===\n");
    ROS_INFO("PITCH (X axis): %lf\n", this->pitch);
    ROS_INFO("ROLL  (Y axis): %lf\n", this->roll);
    ROS_INFO("YAW   (Z axis): %lf\n", this->yaw);
}

void CAR_IMU::run(void){
// Euler Coordinate System
    CAR_IMU::get_quaternion();
    CAR_IMU::get_euler();
    CAR_IMU::print_euler();
}

void imu_callback(const sensor_msgs::Imu::ConstPtr &ori){
    imu_w = ori->orientation.w;
    imu_x = ori->orientation.x;
    imu_y = ori->orientation.y;
    imu_z = ori->orientation.z;

    Car_imu.run();
}
