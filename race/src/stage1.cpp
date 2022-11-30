#include "race/stage1.h"

void run1(void){    
    ROS_INFO("\n==STAGE 1 START==\n");

    YAML::Node pathConfig = YAML::LoadFile(stage1_yaml);

    int count=0;
    double x,y,z;
    for(auto xyz : pathConfig){
        count++;
        x = xyz["xyz"][0].as<double>();
        y = xyz["xyz"][1].as<double>();
        z = xyz["xyz"][2].as<double>();

        MECANUM::moveTo(x,y,z);
        printf("\n=== COUNT: %d ===\n", count);
        printf("SCARA_MODE:%d", SCARA::MODE);
        if(SCARA::MODE){
            printf("APPLE");
            if(count == PHASE_ONE) SCARA::tel_1();
            if(count == PHASE_TWO) SCARA::tel_2();
            if(count == PHASE_THREE) SCARA::cubeoff();
        }

    }
}
