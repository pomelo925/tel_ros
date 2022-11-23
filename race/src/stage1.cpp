#include "race/stage1.h"

void run1(void){    
    ROS_INFO("\n==STAGE 1 START==\n");

    MECANUM::moveTo(0, -96, 0);
    MECANUM::moveTo(35, 0, 0);  // FIRST TAKE
    ROS_INFO("\nTake Photo And Cube (1)...\n");
    SCARA::tel_1();

    MECANUM::moveTo(-45, 0, 0); 
    MECANUM::moveTo(0, 0, 5);  /* cali */
    MECANUM::moveTo(0, -78, 0);
    MECANUM::moveTo(0, 0, 175); /* cali (0,0,180)*/ 
    MECANUM::moveTo(-30, 0, 0);
    MECANUM::moveTo(0, -6, 0);  // SECOND TAKE
    ROS_INFO("\nTake Photo And Cube (2)...\n");
    SCARA::tel_2();
    
    MECANUM::moveTo(0, 50, 0);
    MECANUM::moveTo(0, 0, 180); 
    MECANUM::moveTo(0, -54, 0);   // CUBE OFF 
    ROS_INFO("\nCube Off...\n");
    SCARA::cubeoff();
    
    MECANUM::moveTo(0, -75, 0);  
    MECANUM::moveTo(-15, 0, 0);  
    MECANUM::moveTo(0, 0, 180);  // STAGE 2 START 
}
