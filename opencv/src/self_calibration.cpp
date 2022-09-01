#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

void on_Mouse(int event, int x, int y, int flags, void* param){
    if (event == 0) cout << "x: " << x << "  y: " << y << endl;
}


int main(){
    Mat src = imread("/home/ditrobotics/TEL/src/opencv/src/opencv_frame_0.png");
    cvtColor(src, src, COLOR_RGB2GRAY);
    src.convertTo(src, -1, 2, 0); //increase the contrast by 2
    namedWindow("src");
    setMouseCallback("src", on_Mouse);
    imshow("src", src);
    waitKey();
}