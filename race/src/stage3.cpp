#include "race/stage3.h"
#include "race/microswitch.h"
#include "race/scara.h"

void init3(void);
void run3(void);

int main(int argc, char** argv){
    ROS_INFO("=== STAGE 3 START ===\n");
    
    ros::init(argc, argv, "stage3");
    ros::NodeHandle nh3;
    init3();
    run3();

    ROS_INFO("=== STAGE 3 END ===\n");
}

void init3(void){
    ENCODER::init();
    SWITCH::init();
    SCARA::init();
}

void run3(void){
    ENCODER::moveTo(UP);
    ENCODER::moveTo(DOWN);
    ENCODER::moveTo(LEFT);
    ENCODER::moveTo(TOUCH);
}