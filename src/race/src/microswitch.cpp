#include "race/microswitch.h"

// 初始化變數
void switch_initial(){
    ros::NodeHandle nh_forSwitch;
    switch_subscriber = nh_forSwitch.subscribe("switch_fromSTM", 1,switch_callback);
}

// 微動開關的 callback function
void switch_callback(const std_msgs::Int64MultiArray::ConstPtr &switch_msg){
};


// 建構子
MICRO::MICRO(int a, int b, int c, int d){
    this->A = a;
    this->B = b;
    this->C = c;
    this->D = d;
}

// 判斷有沒有撞牆
bool MICRO::isTouch(){
    if(this->A != switch_sub.data[0]) return false;
    if(this->B != switch_sub.data[1]) return false;
    if(this->C != switch_sub.data[2]) return false;
    if(this->D != switch_sub.data[3]) return false;
    return true;            
}


