#include "race/stage3.h"
#include "race/microswitch.h"
#include "race/scara.h"

void init3(void);
void run3(void);

int main(int argc, char** argv){
    ROS_INFO("=== STAGE 3 START ===\n");

    init3();
    run3();

    ROS_INFO("=== STAGE 3 END ===\n");
}

void init3(void){
    MECANUM::init();
    SWITCH::init();
    SCARA::init();
}

void run3(void){
    MECANUM::moveTo(UP);
    MECANUM::moveTo(DOWN);
    MECANUM::moveTo(LEFT);
    MECANUM::moveTo(TOUCH);
}