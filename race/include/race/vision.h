#ifndef _VISION_H_
#define _VISION_H_

#include "race/scara.h"
#include <opencv2/opencv.hpp>

using namespace cv;

namespace VISION{
    Point2f detect[5];  // 裝辨識出來的點：detect[0]裝E、detect[1]裝T、detect[2]裝L
    bool E_isDetected = false;
    bool T_isDetected = false;
    bool L_isDetected = false;

    bool E_isCatched = false;
    bool T_isCatched = false;
    bool L_isCatched = false;


    void E_image(void);  // 辨識E
    void CTFL_image(void);  // 辨識T、L
    void tf(void);  // 座標轉換

    void taking_photo(void);  //自動拍攝

    /* internal function*/
    Mat E_filter(Mat img);
    void E_contour(Mat original_image, Mat image, double epsilon, \
        int minContour, int maxContour, double lowerBondArea);

    Mat CTFL_filter(Mat img);
    void CTFL_contour(Mat original_image, Mat image, double epsilon, \
        int minContour, int maxContour, double lowerBondArea);
}
#endif
