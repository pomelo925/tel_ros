#include "race/microswitch.h"

void SWITCH::init(void){
    ros::NodeHandle nh_4chassis_witch;
    chassis_switch_subscriber = nh_4chassis_witch.subscribe("chassis_switch_fromSTM", 1,SWITCH::callback);
}

// chassis microswitch callback function
void SWITCH::callback(const std_msgs::Int64MultiArray::ConstPtr &switch_msg){
};

CH_MICRO::CH_MICRO(int fr, int fl, int lf){
    this->fr = fr;
    this->fl = fl;
    this->lf = lf;
}

// determine whether satisfying microswitch condition
bool CH_MICRO::isTouch(){
    if(this->fr != chassis_switch_sub.data[0]) return false;
    if(this->fl != chassis_switch_sub.data[1]) return false;
    if(this->lf != chassis_switch_sub.data[2]) return false;
    return true;            
}
