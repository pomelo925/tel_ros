#include "race/stage3.h"

void run3(void){
    ROS_INFO("\n==STAGE 3 START==\n");
    MECANUM::moveUP(0, 220, 3);
    MECANUM::moveTo(-25, 0, 0);
    ROS_INFO("\nTop Calibration ...\n");

    MECANUM::moveTo(0, 12, 0);  // TOP Calibration
    MECANUM::moveTo(50, 0, 0);
    MECANUM::moveTo(0, 0, 90);
    MECANUM::moveTo(35, 0, 0);

    ROS_INFO("\n==Lateral Movement. ==\n");
    MECANUM::moveTo(0, 50, 0);  // Lateral Movement
    MECANUM::moveTo(35, 0, 0);
    MECANUM::moveTo(0, -50, 0);
    MECANUM::moveTo(50, 0, 0);  // FINISH

    ROS_INFO("\n== FINISHED==\n");
}