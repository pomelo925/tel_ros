#include "race/stage1.h"
#include "race/scara.h"
#include "race/mecanum.h"
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
    MECANUM::moveTo(A_START_PUSH);    
    PUTTER::expansion(true);
    MECANUM::moveTo(A_STOP_PUSH);
    PUTTER::expansion(false);
    MECANUM::moveTo(A_CORNER);
    MECANUM::moveTo(A_CAM);

/**** ..... opencv ..... ****/

    MECANUM::moveTo(B_PUT);
    SCARA::catching(330,0); // 90 degree to right and put off
    MECANUM::moveTo(B_LEAVE);    
    MECANUM::moveTo(B_NEXT_STAGE);
}
