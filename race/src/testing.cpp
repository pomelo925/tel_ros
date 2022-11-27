#include "ros/ros.h"
#include "race/mecanum.h"
#include "race/vision.h"
#include "race/microswitch.h"
#include "race/scara.h"


void init_all_sensors(void){
    MECANUM::init();
    SCARA::init();
}

int main(int argc, char **argv){
    ros::init(argc, argv, "testing");
    ros::NodeHandle nh;
    init_all_sensors();

    SCARA::tel_1();
    // while(ros::ok()) 
    // /* 定點拍照*/
    // printf("    SCARA::movingTo(-330, 0, 2) \n");
    // SCARA::movingTo(-330, 0, 2);

    // while(SCARA::scaraflag!=0) ros::spinOnce();
    // VISION::taking_photo(); 
    
    // /* 辨識*/
    // VISION::E_image();
    // VISION::CTFL_image();
 
    // VISION::tf();
    
    // /*抓*/
    // SCARA::seize();

    return 0;
}
