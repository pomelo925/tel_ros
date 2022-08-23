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


/** Stage 2 initialization **/
/** will NOT be used when integration **/
void init2(void){
    mecanum_init();
    switch_init();
}

/** Stage 2 start running **/
void run2(void){
    moveTo(RED__START);
    moveTo(RED__END);
    moveTo(RED__START);
    moveTo(RED__END);
    moveTo(RED__START);
    moveTo(RED__END);
}