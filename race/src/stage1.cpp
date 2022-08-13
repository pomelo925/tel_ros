#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Point.h>

using namespace std;

ros::Publisher pole; 
ros::Publisher scara;

// 微動開關
bool switch_A, switch_B, switch_C, switch_D;
ros::Subscriber inching_A;
ros::Subscriber inching_B;
ros::Subscriber inching_C;
ros::Subscriber inching_D;
void A_callback();
void B_callback();
void C_callback();
void D_callback();

// 速度積分
ros::Subscriber velX;
ros::Subscriber velY;
ros::Subscriber rot;


// 主函式
void run();

void moveTo(double goal_x, double goal_y, double err);

// 第一階段: 起點 ~ 影像辨識起點
void phase_1();

// 第二階段: 影像辨識 + 夾取方塊 
void phase_2();

// 第三階段: 影像辨識起點 ~ 放置方塊 ~ 終點
void phase_3();

int main(int argc, char** argv){
    ros::init(argc, argv, "phase1");
    run();
    return 0;
}

void phase_1(){
    // P1 -> 微動 -> P2 -> 微動 -> P3 
    moveTo(40, 100, 2);
    moveTo(55, 100, 2); 
    moveTo(40, 140, 2);
    moveTo(20, 140, 2);
}



void run(){
    ros::NodeHandle nh;

    pole = nh.advertise<std_msgs::Int64>("pole", 1);
    scara = nh.advertise<geometry_msgs::Point>("scara",1);
    inching_A = nh.subscribe("inching_A", 1, A_callback);
    inching_B = nh.subscribe("inching_B", 1, B_callback);
    inching_C = nh.subscribe("inching_C", 1, C_callback);
    inching_D = nh.subscribe("inching_D", 1, D_callback);

    phase_1();
    phase_2();
    phase_3();

    return;
}

void A_callback(const std_msgs::Bool::ConstPtr& msg){
    switch_A = msg->data;
}
void B_callback(const std_msgs::Bool::ConstPtr& msg){
    switch_B = msg->data;
}
void C_callback(const std_msgs::Bool::ConstPtr& msg){
    switch_C = msg->data;
} 
void D_callback(const std_msgs::Bool::ConstPtr& msg){
    switch_D = msg->data;
}
