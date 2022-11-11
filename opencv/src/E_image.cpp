#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// 過濾字型，已固定 hsv 參數
Mat filt_letter(Mat img);

// 篩選出好的 contour，標示中心點E
void filt_contour(Mat original_image, Mat image, double epsilon, int minContour, int maxContour, double lowerBondArea);  

int main(){
    const double epsilon = 2;  // DP Algorithm 的參數
    const int minContour = 3;  // 邊數小於 minContour 會被遮罩
    const int maxContour = 6;  // 邊數大於 maxContour 
    const double lowerBondArea = 10;  // 面積低於 lowerBondArea 的輪廓會被遮罩

    string path = "D:\\pomelo\\code\\C++\\OpenCV\\TEL\\tel_ros\\opencv\\opencv_frame_2.png";

    Mat src = imread(path);
    resize(src, src, Size(src.cols/1.5, src.rows/1.5));
    Mat original_image = src.clone();
    src = filt_letter(src);  
    filt_contour(original_image, src, epsilon, minContour, maxContour, lowerBondArea);    
    if(waitKey(0) == 'q') return 0;
}

Mat filt_letter(Mat img){
    Mat img_hsv, mask, result; 
    cvtColor(img, img_hsv, COLOR_BGR2HSV);

// green range
    const int hue_m = 62;
    const int hue_M = 88; 
    const int sat_m = 60; 
    const int sat_M = 255;
    const int val_m = 137;
    const int val_M = 255;  

    Scalar lower(hue_m, sat_m, val_m);
    Scalar upper(hue_M, sat_M, val_M);
    inRange(img_hsv, lower, upper, mask);
    
    result = Mat::zeros(img.size(), CV_8UC3);
    bitwise_and(img, img, result, mask);
    // imshow("Letter Filted", result);
    return result;
}

void filt_contour(Mat original_image, Mat image, double epsilon, int minContour, int maxContour, double lowerBondArea){
    cvtColor(image, image, COLOR_BGR2GRAY);
    threshold(image, image, 40, 255, THRESH_BINARY);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;


// 1) 找出邊緣
    findContours(image, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    imshow("A", image);

    vector<vector<Point>> polyContours(contours.size());  // polyContours 用來存放折線點的集合


// 2) 簡化邊緣： DP Algorithm
    for(size_t i=0; i < contours.size(); i++){
        approxPolyDP(Mat(contours[i]), polyContours[i], epsilon, true);
    }

    Mat dp_image = Mat::zeros(image.size(), CV_8UC3);  // 初始化 Mat 後才能使用 drawContours
    drawContours(dp_image, polyContours, -2, Scalar(255,0,255), 1, 0);
    // imshow("B", dp_image);


// 3) 過濾不好的邊緣，用 badContour_mask 遮罩壞輪廓
    Mat badContour_mask = Mat::zeros(image.size(), CV_8UC3);
    for (size_t a=0; a < polyContours.size(); a++){
        // if 裡面如果是 true 代表該輪廓是不好的，會先被畫在 badContour_mask 上面
        if(polyContours[a].size()<minContour || polyContours[a].size()>maxContour || 
            contourArea(polyContours[a])<lowerBondArea){
            for(size_t b=0; b < polyContours[a].size()-1; b++){
                line(badContour_mask, polyContours[a][b], polyContours[a][b+1], Scalar(0,255,0), 3);
            }
        line(badContour_mask, polyContours[a][0], polyContours[a][polyContours[a].size()-1], Scalar(0,255,0), 1, LINE_AA);
        }
    }


// 4) 進行壞輪廓的遮罩
    Mat dp_optim_v1_image = Mat::zeros(image.size(), CV_8UC3);

    cvtColor(badContour_mask, badContour_mask, COLOR_BGR2GRAY);
    threshold(badContour_mask, badContour_mask, 0, 255, THRESH_BINARY_INV);
    bitwise_and(dp_image, dp_image, dp_optim_v1_image, badContour_mask);
    // imshow("C", dp_optim_v1_image);


// 5) 再從好的邊緣圖中找出邊緣
    cvtColor(dp_optim_v1_image, dp_optim_v1_image, COLOR_BGR2GRAY);
    threshold(dp_optim_v1_image, dp_optim_v1_image, 0, 255, THRESH_BINARY);
    vector<vector<Point>> contours2;
    vector<Vec4i> hierarchy2;

    findContours(dp_optim_v1_image, contours2, hierarchy2, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);


// 6) 簡化好輪廓 DP演算法
    vector<vector<Point>> polyContours2(contours2.size());  // 存放折線點的集合
    Mat dp_image_2 = Mat::zeros(dp_optim_v1_image.size(), CV_8UC3);
    for(size_t i=0; i < contours2.size(); i++){
        approxPolyDP(Mat(contours2[i]), polyContours2[i], epsilon, true);
    }
    drawContours(dp_image_2, polyContours2, -2, Scalar(255,0,255), 1, 0);


// 7) 擬和旋轉矩形 + 標示方塊中心點
    RotatedRect box;  // 旋轉矩形 class
    Point2f vertices[4];  // 旋轉矩形四頂點
    vector<Point> pt;  // 存一個contour中的點集合

    for(int a=0; a<polyContours2.size(); a++){
        pt.clear();
        for(int b=0; b<polyContours2[a].size(); b++){
            pt.push_back(polyContours2[a][b]);
        }

        box = minAreaRect(pt);  // 找到最小矩形，存到 box 中
        box.points(vertices);  // 把矩形的四個頂點資訊丟給 vertices，points()是 RotatedRect 的函式
        for(int i=0; i<4; i++){
            line(dp_image_2, vertices[i], vertices[(i+1)%4], Scalar(0,255,0), 2);  // 描出旋轉矩形
        }
        circle(dp_image_2, (vertices[0]+vertices[1]+vertices[2]+vertices[3])/4, 0, Scalar(0,255,255), 8);  // 繪製中心點
        circle(original_image, (vertices[0]+vertices[1]+vertices[2]+vertices[3])/4, 0, Scalar(0,255,255), 8);  // 放回原圖比較

        putText(dp_image_2, "E", (vertices[0]+vertices[1]+vertices[2]+vertices[3])/4, 1, 3, Scalar(0,0,255), 2);
        putText(original_image, "E", (vertices[0]+vertices[1]+vertices[2]+vertices[3])/4, 1, 3, Scalar(0,0,255), 2);
    }

    // imshow("D", dp_image_2);
    imshow("E", original_image);
}