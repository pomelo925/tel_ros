/** 判斷 CTFL，E 也會被判成 T **/
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// 過濾字型，已固定 hsv 參數
Mat filt_letter(Mat img);

// 篩選出好的 contour，並判斷字母+標示中心點
void filt_contour(Mat original_image, Mat image, double epsilon, int minContour, int maxContour, double lowerBondArea);  

// 印輪廓和點個數在圖上
Mat contours_info(Mat image, vector<vector<Point>> contours);

int main(){
/// 需要調整的變數
    double epsilon = 5.5;  // DP Algorithm 的參數
    int minContour = 4;  // 邊數小於 minContour 會被遮罩
    int maxContour = 8;  // 邊數大於 maxContour 會遮罩
    double lowerBondArea = 20;  // 面積低於 lowerBondArea 的輪廓會被遮罩
///             

    string path = "/home/ditrobotics/TEL/src/opencv/src/opencv_frame_3.png";
    Mat src = imread(path);
    resize(src, src, Size(src.cols, src.rows));
    Mat original_image = src.clone();
    // imshow("original",original_image);
    src = filt_letter(src);  
    filt_contour(original_image, src, epsilon, minContour, maxContour, lowerBondArea); 
    waitKey(0);   
}

Mat filt_letter(Mat img){
    Mat img_hsv, mask, result; 
    cvtColor(img, img_hsv, COLOR_BGR2HSV);

// range 2 (light)
    int hue_m = 82;
    int hue_M = 110; 
    int sat_m = 73; 
    int sat_M = 255;
    int val_m = 255;
    int val_M = 255;  

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
    // imshow("Contours Image (before DP)", image);

    vector<vector<Point>> polyContours(contours.size());  // polyContours 用來存放折線點的集合

// 2) 簡化邊緣： DP Algorithm
    for(size_t i=0; i < contours.size(); i++){
        approxPolyDP(Mat(contours[i]), polyContours[i], epsilon, true);
    }

    Mat dp_image = Mat::zeros(image.size(), CV_8UC3);  // 初始化 Mat 後才能使用 drawContours
    drawContours(dp_image, polyContours, -2, Scalar(255,0,255), 1, 0);
    // imshow("Contours Image (After DP):", dp_image);

// 3) 過濾不好的邊緣，用 badContour_mask 遮罩壞輪廓
    Mat badContour_mask = Mat::zeros(image.size(), CV_8UC3);
    for (size_t a=0; a < polyContours.size(); a++){
        // if 裡面如果是 true 代表該輪廓是不好的，會先被畫在 badContour)mask 上面
        if(polyContours[a].size()<minContour || polyContours[a].size()>maxContour || 
            contourArea(polyContours[a])<lowerBondArea){
            for(size_t b=0; b < polyContours[a].size()-1; b++){
                line(badContour_mask, polyContours[a][b], polyContours[a][b+1], Scalar(0,255,0), 3);
            }
        line(badContour_mask, polyContours[a][0], polyContours[a][polyContours[a].size()-1], Scalar(0,255,0), 1, LINE_AA);
        }
    }

// 進行壞輪廓的遮罩
    Mat dp_optim_v1_image = Mat::zeros(image.size(), CV_8UC3);

    cvtColor(badContour_mask, badContour_mask, COLOR_BGR2GRAY);
    threshold(badContour_mask, badContour_mask, 0, 255, THRESH_BINARY_INV);
    bitwise_and(dp_image, dp_image, dp_optim_v1_image, badContour_mask);
    // imshow("DP image (Optim v1): ", dp_optim_v1_image);


// 4) 再從好的邊緣圖中找出邊緣
    cvtColor(dp_optim_v1_image, dp_optim_v1_image, COLOR_BGR2GRAY);
    threshold(dp_optim_v1_image, dp_optim_v1_image, 0, 255, THRESH_BINARY);
    vector<vector<Point>> contours2;
    vector<Vec4i> hierarchy2;

    findContours(dp_optim_v1_image, contours2, hierarchy2, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

// 5) 簡化好輪廓 DP演算法
    vector<vector<Point>> polyContours2(contours2.size());  // 存放折線點的集合
    Mat dp_image_2 = Mat::zeros(dp_optim_v1_image.size(), CV_8UC3);
    for(size_t i=0; i < contours2.size(); i++){
        approxPolyDP(Mat(contours2[i]), polyContours2[i], epsilon, true);
    }
    drawContours(dp_image_2, polyContours2, -2, Scalar(255,0,255), 1, 0);

    Mat dp_image_text = dp_image_2.clone();
    dp_image_text = contours_info(dp_image_text, polyContours2);
    // imshow("Contours Image (After DP):", dp_image_text);


// 7) 擬和旋轉矩形 + 邊長數量判斷字型 + 標示方塊中心點
    RotatedRect box;  // 旋轉矩形 class
    Point2f vertices[4];  // 旋轉矩形四頂點
    vector<Point> pt;  // 存一個contour中的點集合

    for(int a=0; a<polyContours2.size(); a++){
    // A) 旋轉矩形
        pt.clear();
        for(int b=0; b<polyContours2[a].size(); b++){
            pt.push_back(polyContours2[a][b]);
        }
        box = minAreaRect(pt);  // 找到最小矩形，存到 box 中
        box.points(vertices);  // 把矩形的四個頂點資訊丟給 vertices，points()是 RotatedRect 的函式

        for(int i=0; i<4; i++){
            line(dp_image_2, vertices[i], vertices[(i+1)%4], Scalar(0,255,0), 2);  // 描出旋轉矩形
        }

        // 標示
        circle(dp_image_2, (vertices[0]+vertices[1]+vertices[2]+vertices[3])/4, 0, Scalar(0,255,255), 8);  // 繪製中心點
        circle(original_image, (vertices[0]+vertices[1]+vertices[2]+vertices[3])/4, 0, Scalar(0,255,255), 8);  // 與原圖比較
    
    // B) 判斷字母(用邊長個數篩選)
        if(polyContours2[a].size() == 6){  // L 
            // 標示
            putText(dp_image_2, "L", (vertices[0]+vertices[1]+vertices[2]+vertices[3])/4, 2, 3, Scalar(0, 255, 255), 3);
            putText(original_image, "L", (vertices[0]+vertices[1]+vertices[2]+vertices[3])/4, 2, 1, Scalar(0, 0, 255), 3);
        }

        if(polyContours2[a].size() == 8){  // T、E (此時場上不會有 E)
            // 標示
            putText(dp_image_2, "T", (vertices[0]+vertices[1]+vertices[2]+vertices[3])/4, 2, 3, Scalar(0, 255, 255), 3);
            putText(original_image, "T", (vertices[0]+vertices[1]+vertices[2]+vertices[3])/4, 2, 3, Scalar(0, 0, 255),3);
        }
    }

    // imshow("Contours Filted", dp_image_2);
    imshow("HIGHLIGHT", original_image);
}

Mat contours_info(Mat image, vector<vector<Point>> contours){
    Mat info_image = Mat::zeros(image.size(), CV_8UC3);

// 輪廓數量
    string name1 = "Number of Contours: " + to_string(contours.size());

// 點數量
    int pt_count =0;
    for(size_t a=0; a < contours.size(); a++){
        for(size_t b=0; b < contours[a].size(); b++){
            pt_count ++; 
        }
    }
    string name2 = "Number of Contours Points: " + to_string(pt_count);

    putText(info_image, name1, Point(10,25), 0, 0.8, Scalar(0,255,0), 1, 1, false);
    putText(info_image, name2, Point(10,60), 0, 0.8, Scalar(0,255,0), 1, 1, false);
    drawContours( info_image, contours, -2, Scalar(0,0,255), 1, 0);
    return info_image;
}
