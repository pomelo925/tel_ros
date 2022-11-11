#include "race/stage1.h"
#include "race/scara.h"
#include "race/mecanum.h"
#include "race/vision.h"
#include "race/microswitch.h"

void init1(void);
void run1(void);

int main(int argc, char** argv){
    ROS_INFO("=== STAGE 1 START ===\n");

    init1();
    run1();
    
    ROS_INFO("=== STAGE 1 END ===\n");
}


/** Stage 1 initialization **/
/** will NOT be used when integration **/
void init1(void){
    MECANUM::init();
    SCARA::init();
}


void run1(void){    
/*  1st Seizing */
    MECANUM::moveTo(START_PHOTO_1);  
    VISION::E_image();
    VISION::CTFL_image();
    VISION::point_sort();

/* 2nd Seizing */
    MECANUM::moveTo(START_PHOTO_2);
    VISION::E_image();
    VISION::CTFL_image();
    VISION::point_sort();

}
