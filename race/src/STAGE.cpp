#include "race/stage1.h"
#include "race/stage2.h"
#include "race/stage3.h"

void init_all_sensors();

int main(int argc, char **argv){
    ros::init(argc, argv, "STAGE");
    ros::NodeHandle nh;

    init_all_sensors();
        
    int reset_state=1;
    // nh.getParam("/reset_state",reset_state);

    while (ros::ok()){
        if(reset_state == 1 || reset_state == 0){
            run1(); run2(); run3();
            break;
        }
        
        else if (reset_state == 2){
            run2(); run3();
            break;
        }
        
        else if (reset_state == 3){
            run3();
            break;
        }
    }
}

void init_all_sensors(){
    MECANUM::init();
    SCARA::init();
    SWIITCH::init();
    IMU::init();
}