#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h> 
#include <sensor_msgs/image_encodings.h>  
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

// global variables
image_transport::Publisher pub;
sensor_msgs::ImagePtr msg;

// 更完整的過濾字型 Image -- OptimV2
int main(int argc, char** argv){
    ros::init(argc, argv, "E_cam_pub");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh); 
    pub = it.advertise("E_cam", 1);

    Mat src;
    VideoCapture cap(1);

/// 需要調整的變數 ///
    double epsilon = 3;  // DP Algorithm 的參數
    int minContour = 5;  // 邊數小於 minContour 會被遮罩
    int maxContour = 20;  // 邊數大於 maxContour 會遮罩
    double lowerBondArea = 45;  // 面積低於 lowerBondArea 的輪廓會被遮罩
///             

    if(!cap.isOpened()) ROS_ERROR_STREAM("Cannot open capture\n");
    while(ros::ok()){
        bool ret = cap.read(src);
        if(!ret){
            ROS_ERROR_STREAM("Cant receive frame\n");
            break;
        }

        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src).toImageMsg();
        pub.publish(msg);
        if(waitKey(1) == 'q') break;       
    }
    cap.release();
    return 0;
}