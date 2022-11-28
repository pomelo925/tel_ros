#include "ros/ros.h"
#include "race/mecanum.h"
#include "race/vision.h"
#include "race/microswitch.h"
#include "race/scara.h"


void init_all_sensors(void){
    MECANUM::init();
    SCARA::init();
}

int main(int argc, char **argv){
    ros::init(argc, argv, "testing");
    ros::NodeHandle nh;
    init_all_sensors();

    // SCARA::tel_1();

    // MECANUM::moveTo(0,50,0);
    //   MECANUM::moveTo(40,0,0);
    //     MECANUM::moveTo(-40,-50,0);
          MECANUM::moveTo(0,0,360);  
    return 0;
}
