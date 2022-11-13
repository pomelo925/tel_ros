#include "race/stage3.h"

void run3(void){
    MECANUM::moveTo(UP);
    MECANUM::moveTo(DOWN);
    MECANUM::moveTo(LEFT);
    MECANUM::moveTo(TOUCH);
}