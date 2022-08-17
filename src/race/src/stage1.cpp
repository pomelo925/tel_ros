#include "race/stage1_lib.h"

int main(int argc, char **argv){
    ros::init(argc, argv , "stage1");
    ROS_INFO("Starting");
    run(); 
}