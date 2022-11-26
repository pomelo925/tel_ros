#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include "ros/ros.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv){
    ros::init(argc, argv, "read");
    ros::NodeHandle nh;

    VideoCapture cap(0);
    Mat img;

    if(!cap.isOpened()) cout << "Cannot open capture\n";

    while(true){
        bool ret = cap.read(img);
        if(!ret){
            cout<<"cant receive frame\n";
            break;
        }

        imshow("Image", img);
        if(waitKey(1) == 27) break;

    }
    return 0;
}