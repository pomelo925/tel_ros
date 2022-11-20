#include "race/stage2.h"

void run2(void){
    ROS_INFO("\n==STAGE 2 START==\n");

    MECANUM::moveTo(20, 0, 0);
    MECANUM::moveTo(0, 0, -5);
    MECANUM::moveTo(0, 80, 0);
    MECANUM::moveTo(-14, 0, 0);  // 20 POINT Calibration
    MECANUM::moveTo(0, 60, 0);
    MECANUM::moveTo(-27.5, 0, 0);
    MECANUM::moveTo(0, 57.5, 0);  
    MECANUM::moveTo(15, 0, 0);  // 40 POINT Calibration
    MECANUM::moveTo(0, 52.5, 0);    
    MECANUM::moveTo(25, 0, 0);
    MECANUM::moveTo(0, 50, 0);
    MECANUM::moveTo(-7, 0, 0);  // 60 POINT Calibration  
    MECANUM::moveTo(0, 50, 0);
    MECANUM::moveTo(-10, 0, 0);
    
}