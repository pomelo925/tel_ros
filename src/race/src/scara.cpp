#include "race/scara.h"

void SCARA::init(void){
    ros::NodeHandle nh_4scara;
    putter_pub = nh_4scara.advertise<std_msgs::Int64>("putter_toSTM", 1); 
    scara_pub = nh_4scara.advertise<geometry_msgs::Point>("scara_toSTM", 1);
}

void SCARA::catching(double x, double y){
    scara_cor.x = x;
    scara_cor.y = y;
    scara_pub.publish(scara_cor);
}


void PUTTER::expansion(bool open){
    status.data = (open == true)? 1:0;
    putter_pub.publish(status);
}


