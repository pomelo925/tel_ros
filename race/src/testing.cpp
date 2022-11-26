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


    VISION::taking_photo();
    VISION::E_image();
    VISION::CTFL_image();
    VISION::tf();

    SCARA::seize();

    return 0;
}
