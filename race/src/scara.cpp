#include "race/scara.h"
#include "race/vision.h"

namespace SCARA{
    double vision_x, vision_y, scaraflag;
}

void SCARA::init(void){
    ros::NodeHandle nh_4scara;
    scara_pub = nh_4scara.advertise<geometry_msgs::Point>("scara_toSTM", 1);
    scara_sub = nh_4scara.subscribe("scaraflag_fromSTM", 1, SCARA::callback);
}

void SCARA::callback(const std_msgs::Float64::ConstPtr &flag){
    scaraflag = flag->data;
}

void SCARA::movingTo(double x, double y, double z){
    scara_cor.x = x;
    scara_cor.y = y;
    scara_cor.z = z;
    scara_pub.publish(scara_cor);
    while(scaraflag!=0);
}

void SCARA::tel_1(void){
    SCARA::movingTo(0, -50, 1);
        ros::Duration(1).sleep();
        while(scaraflag!=0.);
    
    SCARA::movingTo(-330, 0, 2);
        ros::Duration(1).sleep();
        while(scaraflag!=0.);printf("\nBBBBBBBBBBBBBBBBB");
    VISION::taking_photo();

    // SCARA::movingTo(vision_x, vision_y, 3);
}


void SCARA::tel_2(void){
    SCARA::movingTo(-330, 0, 2);
        ros::Duration(1).sleep();
        while(scaraflag!=0.);printf("\nAAAAAAAAAAAA");

    VISION::taking_photo();

    // SCARA::movingTo(vision_x, vision_y, 3);
}


void SCARA::cubeoff(void){
    SCARA::movingTo(0, 330, 4); 
        ros::Duration(3).sleep();
        while(scaraflag!=0.);printf("\nAAAAAAAAAAAA");
    
    SCARA::movingTo(0, -50, 2); 
        ros::Duration(1).sleep();
        while(scaraflag!=0.);printf("\nAAAAAAAAAAAA");
}
