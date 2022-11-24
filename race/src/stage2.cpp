#include "race/stage2.h"

void run2(void){
    ROS_INFO("\n==STAGE 2 START==\n");
    MECANUM::moveTo(22, 65, -2);
    // MECANUM::moveTo(0, 80, 0);
        
    ROS_INFO("\n20 Points Calibration ...\n");
    MECANUM::moveTo(-15, 65, 2);  // 20 POINT Calibration
    MECANUM::moveTo(-35, 55, 4); 
        
    ROS_INFO("\n40 Points Calibration ...\n");
    MECANUM::moveTo(-15, 50, -10);  // 40 POINT Calibration
    MECANUM::moveTo(50, 85, 0);
        
    ROS_INFO("\n60 Points Calibration ...\n");
    MECANUM::moveTo(-10, 35, 0);  // 60 POINT Calibration  
    MECANUM::moveTo(-20, 15, 0);
    
}