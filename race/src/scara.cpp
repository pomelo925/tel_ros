#include "race/scara.h"

void SCARA::init(void){
    ros::NodeHandle nh_4scara;
    scara_pub = nh_4scara.advertise<geometry_msgs::Point>("scara_toSTM", 1);
}

void SCARA::movingTo(double x, double y){
    scara_cor.x = x;
    scara_cor.y = y;
    scara_pub.publish(scara_cor);
}
