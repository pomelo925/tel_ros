#include "race/stage1.h"
#include "race/scara.h"
#include "race/encoder.h"
#include "race/microswitch.h"

void init1(void);
void run1(void);

int main(int argc, char** argv){
    ros::init(argc, argv, "stage1");
    ROS_INFO("=== STAGE 1 START ===\n");
    
    ros::NodeHandle nh1;
    ROS_INFO("test");
    init1();
    ROS_INFO("test");
    run1();
    ROS_INFO("=== STAGE 1 END ===\n");
}


/** Stage 1 initialization **/
/** will NOT be used when integration **/
void init1(void){
}


void run1(void){    
    ENCODER::moveTo(A_START_PUSH);    
    PUTTER::expansion(true);
    ENCODER::moveTo(A_STOP_PUSH);
    PUTTER::expansion(false);
    ENCODER::moveTo(A_CORNER);
    ENCODER::moveTo(A_CAM);

/**** ..... opencv ..... ****/

    ENCODER::moveTo(B_PUT);
    SCARA::catching(330,0); // 90 degree to right and put off
    ENCODER::moveTo(B_LEAVE);    
    ENCODER::moveTo(B_NEXT_STAGE);
}
