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
        /* 定點拍照*/
            printf(" SCARA::movingTo(-330, 0, 2) \n");
            SCARA::movingTo(-330, 0, 2);
            VISION::taking_photo();

            VISION::E_image();
            VISION::tf();
            SCARA::seize();
            break;


        case 2:
            /* 定點拍照*/
            printf("    SCARA::movingTo(-330, 0, 2) \n");
            SCARA::movingTo(-330, 0, 2);
            VISION::taking_photo();
            
            /* 辨識*/
            VISION::CTFL_image();
            VISION::tf();
            SCARA::seize();
            break;

        case 3:
            /* 定點拍照*/
            printf("    SCARA::movingTo(-330, 0, 2) \n");
            SCARA::movingTo(-330, 0, 2);
            printf("   taking photo) \n");
            VISION::taking_photo();
            
            /* 辨識*/
            VISION::E_image();
            VISION::tf();
            SCARA::seize();


            /* 定點拍照*/
            printf("    SCARA::movingTo(-330, 0, 2) \n");
            SCARA::movingTo(-330, 0, 2);
            VISION::taking_photo();
            
            /* 辨識*/
            VISION::CTFL_image();
            VISION::tf();    
            SCARA::seize();
            break;
        
        default:
            break;

    }

    printf("\nfinish !!");
    return 0;
}
