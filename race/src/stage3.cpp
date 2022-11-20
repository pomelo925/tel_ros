#include "race/stage3.h"

void run3(void){
    MECANUM::moveTo(0, 210, 0);
    MECANUM::moveTo(-25, 0, 0);
    MECANUM::moveTo(0, 12, 0);  // TOP Calibration
    MECANUM::moveTo(50, 0, 0);
    MECANUM::moveTo(0, 0, 90);
    MECANUM::moveTo(35, 0, 0);
    MECANUM::moveTo(0, 50, 0);  // Lateral Movement
    MECANUM::moveTo(35, 0, 0);
    MECANUM::moveTo(0, -50, 0);
    MECANUM::moveTo(50, 0, 0);  // FINISH
}