#include "race/run.h"

void init_all_sensors();

int main(int argc, char **argv){
    ros::init(argc, argv, "run");
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
    
    return EXIT_SUCCESS;
}

void init_all_sensors(void){
    MECANUM::init();
    SCARA::init();
    SWITCH::init();
    IMU::init();
}