#include "race/imu.h"

CAR_IMU Car_imu;

void imu_init(void){
    ros::NodeHandle nh_4imu;
    imu_subscriber = nh_4imu.subscribe("/imu/data", 1, imu_callback);
}

void CAR_IMU::imu_2_quaternion(double w, double x, double y, double z){
    this->Q.w = w;
    this->Q.x = x;
    this->Q.y = y;
    this->Q.z = z;
}

EulersAngles CAR_IMU::quaternion_2_euler(void) {
    EulersAngles euler;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (Q.w * Q.x + Q.y * Q.z);
    double cosr_cosp = 1 - 2 * (Q.x * Q.x + Q.y * Q.y);
    euler.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (Q.w * Q.y - Q.z * Q.x);
    if (std::abs(sinp) >= 1)
        euler.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        euler.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (Q.w * Q.z + Q.x * Q.y);
    double cosy_cosp = 1 - 2 * (Q.y * Q.y + Q.z * Q.z);
    euler.yaw = std::atan2(siny_cosp, cosy_cosp);

    return euler;
}


void CAR_IMU::print_euler(void){
    ROS_INFO("=== Quaternion Coordinate ===\n");
    ROS_INFO("w : %lf\n", this->Q.w);
    ROS_INFO("x : %lf\n", this->Q.x);
    ROS_INFO("y : %lf\n", this->Q.y);
    ROS_INFO("z : %lf\n\n", this->Q.z);

    ROS_INFO("=== Euler Coordinate ===\n");
    ROS_INFO("PITCH (X axis): %lf\n", this->E.pitch);
    ROS_INFO("ROLL  (Y axis): %lf\n", this->E.roll);
    ROS_INFO("YAW   (Z axis): %lf\n\n", this->E.yaw);
}

void CAR_IMU::run(void){
    this->E = CAR_IMU::quaternion_2_euler();
    CAR_IMU::print_euler();
}

void imu_callback(const sensor_msgs::Imu::ConstPtr &ori){
    Car_imu.imu_2_quaternion(ori->orientation.w, ori->orientation.x, ori->orientation.y, ori->orientation.z);
    Car_imu.run();
}
