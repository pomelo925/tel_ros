#include "ros/ros.h"
#include "race/mecanum.h"
#include "race/vision.h"
#include "race/microswitch.h"
#include "race/scara.h"

void init_all_sensors(void){
    MECANUM::init();
    SCARA::init();
    // SWITCH::init();
}

int main(int argc, char **argv){
    ros::init(argc, argv, "testing");
    ros::NodeHandle nh;
    init_all_sensors();
    
    // VISION::taking_photo();
    // VISION::E_image();
    // VISION::CTFL_image();

    double x,y,z;
    while(1){
        std::cin>>x>>y>>z;
        SCARA::movingTo(x,y,z);
        printf("apple\n");
    }

    return 0;
}

// #include <opencv2/imgcodecs.hpp>
// #include <opencv2/highgui.hpp>
// #include <opencv2/imgproc.hpp>
// #include <iostream>

// using namespace std;
// using namespace cv;

// int main(){
//     VideoCapture cap(0);
//     Mat img;

//     if(!cap.isOpened()){
//         cout << "Cannot open capture\n";
//     }

//     while(true){
//         bool ret = cap.read(img);
//         imshow("Image", img);

//         if(waitKey(1) == 27) break;
//         while(waitKey(1) != 'a');
//     }
//     return 0;
// } 