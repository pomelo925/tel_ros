#include "ros/ros.h"
#include "race/mecanum.h"
#include "race/vision.h"
#include "race/microswitch.h"
#include "race/scara.h"


void init_all_sensors(void){
    MECANUM::init();
    SCARA::init();
    VISION::init();
}

int main(int argc, char **argv){
    ros::init(argc, argv, "testing");
    ros::NodeHandle nh;
    init_all_sensors();
    
    int test_phase; 
    nh.getParam("test_phase", test_phase);

    switch(test_phase){
        case 1:
            SCARA::tel_1();
            break;


        case 2:

            while(ros::ok()){
                double x,y,z;
                printf("enter: ");
                std::cin >>x>>y>>z;
                SCARA::movingTo(x,y,z);
            }
            break;

        case 3:
            printf("    SCARA::movingTo(-330, 0, 2) \n");
            SCARA::movingTo(-330, 0, 2);
            VISION::taking_photo();
            
            /* 辨識*/
            VISION::E_image();
            VISION::tf();
            SCARA::seize();
            break;
        
        default:
            break;

    }

    printf("\nfinish !!");
    return 0;
}
