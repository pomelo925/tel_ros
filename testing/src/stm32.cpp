#include <iostream>
#include "ros/ros.h"
#include <std_msgs/Int64.h>

using namespace std;

void callback(const std_msgs::Int64::ConstPtr &msg);
std_msgs::Int64 tim;

int main(int argc, char** argv){
    ros::init(argc, argv, "count");
    ros::NodeHandle nh;

    tim.data =0 ;
    ros::Publisher pub = nh.advertise<std_msgs::Int64>("counting", 1);
    ros::Subscriber sub = nh.subscribe("times", 1, callback);

    std_msgs::Int64 hello;
    hello.data = 0;

    while(ros::ok()){
        pub.publish(hello);
        hello.data++;
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("terminate ~");
    return 0;
}

void callback(const std_msgs::Int64::ConstPtr &msg){
    tim.data = msg->data;
    ROS_INFO("data: %ld\n", tim.data);
}