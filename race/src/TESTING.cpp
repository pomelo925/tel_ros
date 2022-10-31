#include "ros/ros.h"
#include "race/mecanum.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "testing");
    
    ROS_INFO("=== TESTING START ===\n");
    MECANUM::init();
    
    while(ros::ok()){
        double x, y, z;
        std::cout << "enter goal: x y z\n";
        std::cin >> x >> y >> z;
        if (x==-1 && y==-1 && z==-1) break;
        MECANUM::moveTo(x,y,z);
    }
    ROS_INFO("=== TESTING END ===\n");

}