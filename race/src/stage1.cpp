#include "race/stage1.h"

void run1(void){    
    ROS_INFO("\n==STAGE 1 START==\n");

    MECANUM::moveTo(35, -100, -1);
    
    SCARA::tel_1();  // 辨識方塊一

    MECANUM::moveTo(-55, 8, 5);

    MECANUM::moveTo(0, -80, 0);
    MECANUM::moveTo(0, 0, 180);
    MECANUM::moveTo(-30, 0, 0);
    MECANUM::moveTo(0, -10, 0);


    if( VISION::E_isDetected == false || VISION::L_isDetected == false || VISION::T_isDetected == false) 
    {SCARA::tel_2();}  // 辨識方塊二
    
    MECANUM::moveTo(0, 30, 0);
    MECANUM::moveTo(0, 0, 175); 
    MECANUM::moveTo(-5, -85, 0);
    
    SCARA::cubeoff();  // 放方塊
    MECANUM::moveTo(0, 0, 180); 

    MECANUM::moveTo(20, 0, 0);  
    MECANUM::moveTo(0, 78, 0);  
    MECANUM::moveTo(15, 0, 0);  // STAGE 2 START
}
