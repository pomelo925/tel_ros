#include "race/run.h"

void init_all_sensors(ros::NodeHandle nh);

int main(int argc, char **argv){
    ros::init(argc, argv, "run");
    ros::NodeHandle nh_4run;
        
    nh_4run.getParam("reset_state", reset_state);
    ROS_INFO("State Now: %d", reset_state);

    init_all_sensors(nh_4run);

    switch(reset_state){
        case 0:
            while(nh_4run.ok()) ros::spinOnce();
            
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

 
void init_all_sensors(ros::NodeHandle nh){
    MECANUM::init(nh);
    SCARA::init(nh);
    // SWITCH::init();
    // IMU::init();
}

