#include "race/stage2.h"

void init2(void);
void run2(void);

int main(int argc, char** argv){
    ROS_INFO("=== STAGE 2 START ===\n");

    ros::init(argc, argv, "stage2");
    ros::NodeHandle nh2;
    init2();
    run2();

    ROS_INFO("=== STAGE 2 END ===\n");
}


/** Stage 2 initialization **/
/** will NOT be used when integration **/
void init2(void){
    ENCODER::init();
    SWITCH::init();
}

/** Stage 2 start running **/
void run2(void){
    ENCODER::moveTo(RED__START);
    ENCODER::moveTo(RED__END);
    ENCODER::moveTo(GREEN__START);
    ENCODER::moveTo(GREEN__END);
    ENCODER::moveTo(BLUE__START);
    ENCODER::moveTo(BLUE__END);
}