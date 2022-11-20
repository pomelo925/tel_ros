#include "race/stage1.h"

void run1(void){    
    ROS_INFO("\n==STAGE 1 START==\n");
    
    MECANUM::moveTo(0, -96, 0);
    MECANUM::moveTo(35, 0, 0);  // FIRST TAKE
    MECANUM::moveTo(-45, 0, 0); 
    MECANUM::moveTo(0, -78, 0);
    MECANUM::moveTo(0, 0, 180);
    MECANUM::moveTo(-28.5, 0, 0);
    MECANUM::moveTo(0, -3, 0);  // SECOND TAKE
    MECANUM::moveTo(0, 50, 0);
    MECANUM::moveTo(0, 0, 180); 
    MECANUM::moveTo(0, -54, 0);  // CUBE OFF 
    MECANUM::moveTo(0, -75, 0);  
    MECANUM::moveTo(-18.5, 0, 0);  
    MECANUM::moveTo(0, 0, 180);  // STAGE 2 START 
}
