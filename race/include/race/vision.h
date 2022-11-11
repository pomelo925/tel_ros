#ifndef _VISION_H_
#define _VISION_H_

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;

namespace VISION{
    Point2f detected[10];  // 裝辨識出來的點，做處理

    void E_image();  // 辨識E
    void CTFL_image();  // 辨識T、L
    void point_sort();  // 應付 scara 夾取的先後問題


    /* internal function*/
    Mat E_filter(Mat img);
    void E_contour(Mat original_image, Mat image, double epsilon,\ 
        int minContour, int maxContour, double lowerBondArea);

    Mat CTFL_filter(Mat img);
    void CTFL_contour(Mat original_image, Mat image, double epsilon,\
        int minContour, int maxContour, double lowerBondArea);
}

#endif
