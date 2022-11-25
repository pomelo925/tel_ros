#include "race/run.h"

void init_all_sensors();

int main(int argc, char **argv){
    ros::init(argc, argv, "run");
    ros::NodeHandle nh;
        
    int reset_state=1;
    // nh.getParam("/reset_state", reset_state);
    ROS_INFO("State Now: %d", reset_state);

    init_all_sensors();

    switch(reset_state){
        case 0:
        case 1:
            run1(); run2(); run3();
            break;
            
        case 2:
            run2(); run3();
            break;
        
        case 3:
            run3(); 
            break;
    }
    return EXIT_SUCCESS;
}

 
void init_all_sensors(void){
    MECANUM::init();
    SCARA::init();
    // SWITCH::init();
    // IMU::init();
}

