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
    MECANUM::init();
    SWITCH::init();
}

/** Stage 2 start running **/
void run2(void){
    MECANUM::moveTo(RED__START);
    MECANUM::moveTo(RED__END);
    MECANUM::moveTo(GREEN__START);
    MECANUM::moveTo(GREEN__END);
    MECANUM::moveTo(BLUE__START);
    MECANUM::moveTo(BLUE__END);
}