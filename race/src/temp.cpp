#include "race/imu.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "stage2");

    imu_init();
    
    while(ros::ok()){
        ros::spinOnce();
    }
}
