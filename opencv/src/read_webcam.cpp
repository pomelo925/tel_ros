#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main(){
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