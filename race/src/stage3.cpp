#include "race/stage3.h"

void run3(void){
    ROS_INFO("\n==STAGE 3 START==\n");

    YAML::Node pathConfig = YAML::LoadFile(stage3_yaml);

    int count=0;
    double x,y,z;
    for(auto xyz : pathConfig){
        count++;
        x = xyz["xyz"][0].as<double>();
        y = xyz["xyz"][1].as<double>();
        z = xyz["xyz"][2].as<double>();


        if(count == 1) MECANUM::moveUP(x,y,z);
        else MECANUM::moveTo(x,y,z);
        
    }
}