#include "race/stage3.h"

void run3(void){
        ROS_INFO("\n==STAGE 3 START==\n");
    MECANUM::moveTo(UP);
    MECANUM::moveTo(DOWN);
    MECANUM::moveTo(LEFT);
    MECANUM::moveTo(TOUCH);
}