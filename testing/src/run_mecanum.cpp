#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "iostream"

void vel_callback(const geometry_msgs::Point::ConstPtr &msg){};

int main(int argc, char **argv){
    ros::init(argc, argv, "run_mecanum");
    ros::NodeHandle nh;

    geometry_msgs::Point vel;
    ros::Publisher vel_publisher = nh.advertise<geometry_msgs::Point>("mecanum_toSTM", 1);
    ros::Subscriber vel_subscriber = nh.subscribe("mecanum_fromSTM", 1, vel_callback);
    
    double x, y, z;
    while(ros::ok()){
        std::cin >> x >> y >> z;
        vel.x = x;
        vel.y = y;
        vel.z = z;
        vel_publisher.publish(vel);
        // ros::Duration(0.5).sleep();
    }
    return 0;
}

