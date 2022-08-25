#include "race/imu.h"
#include <cmath>

CAR_IMU Car_imu;

void imu_init(void){
    ros::NodeHandle nh_4imu;
    imu_subscriber = nh_4imu.subscribe("/imu/data", 1, imu_callback);
}

void CAR_IMU::get_quaternion(void){
    this->w = imu_w;
    this->x = imu_x;
    this->y = imu_y;
    this->z = imu_z;
}

/* WikiPedia -- problematic with BNO055 */
// void CAR_IMU::get_euler(void) {
//     // roll (x-axis rotation)
//     double sinr_cosp = 2 * (w * x + y * z);
//     double cosr_cosp = 1 - 2 * (x * x + y * y);
//     roll = std::atan2(sinr_cosp, cosr_cosp);

//     // pitch (y-axis rotation)
//     double sinp = 2 * (w * y - z * x);
//     if (std::abs(sinp) >= 1)
//         pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
//     else
//         pitch = std::asin(sinp);

//     // yaw (z-axis rotation)
//     double siny_cosp = 2 * (w * z + x * y);
//     double cosy_cosp = 1 - 2 * (y * y + z * z);
//     yaw = std::atan2(siny_cosp, cosy_cosp);
// }

void CAR_IMU::get_euler(void){
    // roll (x-axis rotation)
    double q32 = 2 * (y*z + w*x);
    double q33 = 1 - 2 * (x*x + y*y);
    roll = atan(q32/q33);

    // pitch (y-axis rotation)
    double q31 = 2 * (x*z - w*y);
    pitch = asin(-q31);

    // yaw (z-axis rotation)
    double q21 = 2 * (x*y - w*z);
    double q11 = 1 - 2 * (y*y + z*z);
    yaw = atan(q21/q11);
}


void CAR_IMU::print_euler(void){
    ROS_INFO("=== Quaternion Coordinate ===\n");
    ROS_INFO("w : %lf\n", this->w);
    ROS_INFO("x : %lf\n", this->x);
    ROS_INFO("y : %lf\n", this->y);
    ROS_INFO("z : %lf\n\n", this->z);

    ROS_INFO("=== Euler Coordinate ===\n");
    ROS_INFO("PITCH (X axis): %lf\n", this->pitch);
    ROS_INFO("ROLL  (Y axis): %lf\n", this->roll);
    ROS_INFO("YAW   (Z axis): %lf\n\n", this->yaw);
}

void CAR_IMU::run(void){
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
