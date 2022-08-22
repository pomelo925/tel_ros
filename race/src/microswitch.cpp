#include "race/microswitch.h"

// declare NodeHandle and pub/sub
void switch_init(){
    ros::NodeHandle nh_4chassis_witch;
    chassis_switch_subscriber = nh_4chassis_witch.subscribe("chassis_switch_fromSTM", 1,switch_callback);
}

// chassis microswitch callback function
void switch_callback(const std_msgs::Int64MultiArray::ConstPtr &switch_msg){
};

CH_MICRO::CH_MICRO(int a, int b, int c, int d){
    this->A = a;
    this->B = b;
    this->C = c;
    this->D = d;
}

// determine whether satisfying microswitch condition
bool CH_MICRO::isTouch(){
    if(this->A != chassis_switch_sub.data[0]) return false;
    if(this->B != chassis_switch_sub.data[1]) return false;
    if(this->C != chassis_switch_sub.data[2]) return false;
    if(this->D != chassis_switch_sub.data[3]) return false;
    return true;            
}
