#include "race/microswitch.h"

void SWITCH::init(void){
    ros::NodeHandle nh_4chassis_witch;
    microswitch_sub = nh_4chassis_witch.subscribe("microswitch_fromSTM", 1,SWITCH::callback);
}

// chassis microswitch callback function
void SWITCH::callback(const geometry_msgs::Point::ConstPtr &switch_msg){
};

CH_MICRO::CH_MICRO(int fr, int fl, int lf){
    this->fr = fr;
    this->fl = fl;
    this->lf = lf;
}

// determine whether satisfying microswitch condition
bool CH_MICRO::isTouch(){
    if(this->fr != chassis_switch_sub.x) return false;
    if(this->fl != chassis_switch_sub.y) return false;
    if(this->lf != chassis_switch_sub.z) return false;
    return true;            
}
