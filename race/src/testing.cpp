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
    
    int test_phase; 
    nh.getParam("test_phase", test_phase);

    printf("  test_phase: %d\n", test_phase);
    printf("  test_phase: %d\n", test_phase);
    printf("  test_phase: %d\n", test_phase);

    switch(test_phase){
        case 1:
        /* 歸零*/
            printf("    SCARA::movingTo(0, -50, 1) \n");
            SCARA::movingTo(0, -50, 1);
            
        /* 定點拍照*/
            printf("    SCARA::movingTo(-330, 0, 2) \n");
            SCARA::movingTo(-330, 0, 2);
            VISION::taking_photo();
            break;


        case 2:
        /* 定點拍照*/
            printf("    SCARA::movingTo(-330, 0, 2) \n");
            SCARA::movingTo(-330, 0, 2);
            VISION::taking_photo();
            break;

        case 3:
            break;
        
        default:
            break;

    }

    return 0;
}
