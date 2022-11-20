#include "ros/ros.h"
#include "race/mecanum.h"
#include "race/vision.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "testing");
    MECANUM::init();

    // MECANUM::moveTo(0, -96, 0);
    // MECANUM::moveTo(35, 0, 0);  // FIRST TAKE
    // MECANUM::moveTo(-45, 0, 0); 
    // MECANUM::moveTo(0, -78, 0);
    // MECANUM::moveTo(0, 0, 180);
    // MECANUM::moveTo(-28.5, 0, 0);
    // MECANUM::moveTo(0, -3, 0);  // SECOND TAKE
    // MECANUM::moveTo(0, 50, 0);
    // MECANUM::moveTo(0, 0, 180); 
    // MECANUM::moveTo(0, -54, 0);  // CUBE OFF 
    // MECANUM::moveTo(0, -75, 0);  
    // MECANUM::moveTo(-18.5, 0, 0);  
    // MECANUM::moveTo(0, 0, 180);  // STAGE 2 START 

    // MECANUM::moveTo(20, 0, 0);
    // MECANUM::moveTo(0, 80, 0);
    // MECANUM::moveTo(-14, 0, 0);  // 20 POINT Calibration
    // MECANUM::moveTo(0, 60, 0);
    // MECANUM::moveTo(-27.5, 0, 0);
    // MECANUM::moveTo(0, 57.5, 0);  
    // MECANUM::moveTo(12, 0, 0);  // 40 POINT Calibration
    // MECANUM::moveTo(0, 52.5, 0);    
    // MECANUM::moveTo(25, 0, 0);
    // MECANUM::moveTo(0, 50, 0);
    // MECANUM::moveTo(-7, 0, 0);  // 60 POINT Calibration  
    // MECANUM::moveTo(0, 50, 0);
    // MECANUM::moveTo(-10, 0, 0);

    MECANUM::moveTo(0, 210, 0);
    MECANUM::moveTo(-25, 0, 0);
    MECANUM::moveTo(0, 12, 0);  // TOP Calibration
    MECANUM::moveTo(50, 0, 0);
    MECANUM::moveTo(0, 0, 90);
    MECANUM::moveTo(35, 0, 0);
    MECANUM::moveTo(0, 50, 0);  // Lateral Movement
    MECANUM::moveTo(35, 0, 0);
    MECANUM::moveTo(0, -50, 0);
    MECANUM::moveTo(50, 0, 0);  // FINISH



    while (ros::ok()){
        double x, y, z;
        std::cout << "enter goal: x y z\n";
        std::cin >> x >> y >> z;
        if (x == -1 && y == -1 && z == -1) break;
        MECANUM::moveTo(x, y, z);
    }
    ROS_INFO("=== TESTING END ===\n");
}