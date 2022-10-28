#include "ros/ros.h"
#include "race/encoder.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "testing");
    
    ROS_INFO("=== TESTING START ===\n");
    ENCODER::init();
    ENCODER::moveTo(0,40,0);
    ROS_INFO("=== TESTING END ===\n");

}