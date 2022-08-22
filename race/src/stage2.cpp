#include "race/stage2.h"

void init2(void);
void run2(void);

int main(int argc, char** argv){
    ros::init(argc, argv, "stage2");

    ROS_INFO("=== STAGE 2 START ===\n");
    init2();
    run2();
    ROS_INFO("=== STAGE 2 END ===\n");
}

void init2(void){
    mecanum_init();
    switch_init();
}

void run2(void){
    moveTo(20,10);
}