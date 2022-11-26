#include "race/vision.h"
#include "math.h"
#include "ros/ros.h"

#define PATH "/home/ditrobotics/TEL/src/race/src/auto.png"
#define csv_file "/home/ditrobotics/TEL/src/race/src/coordinate.csv"

void VISION::tf(void){
    FILE *file;
    file=fopen(csv_file, "r");

    int records=0;
    int row=0;
    do{
        records = fscanf(file, "%f%f%f%f\n", &COR[row].x_pixel, &COR[row].y_pixel, &COR[row].x_scara, &COR[row].y_scara);
        if(records==4) row++;
        else return;
    } while(!feof(file));
    fclose(file);
    file = NULL;

    if(T_isDetected && !T_isCatched) VISION::detect[0]=nearest_scara_point(VISION::detect[0]);
    if(E_isDetected && !E_isCatched) VISION::detect[1]=nearest_scara_point(VISION::detect[1]);
    if(L_isDetected && !L_isCatched) VISION::detect[2]=nearest_scara_point(VISION::detect[2]);

    for(int i=0; i<5; i++) std::cout << "(tf) detect["<<i<<"]: " << VISION::detect[i]<<std::endl; 
}

Point2f VISION::nearest_scara_point(Point2f input){
    Point2f temp;  

    float distance[1121]={0};

    for(int i=0; i<1121; i++) 
    distance[i]=sqrt( (input.x-COR[i].x_pixel)*(input.x-COR[i].x_pixel) \
     + (input.y-COR[i].y_pixel)*(input.y-COR[i].y_pixel));

    int min = 0;
    for(int i=0; i<1121; i++) if(distance[i]<distance[min]) min=i;
    
    temp.x = COR[min].x_scara;
    temp.y = COR[min].y_scara;
    return temp;
}


void VISION::taking_photo(void){
    VideoCapture cap(0);
    while(!cap.isOpened())  printf("Not opened!\n");
    printf(" Waiting Camera Stable... \n");
    Mat img;

    int count=0;
    ros::Rate loop_rate(10);
    while(ros::ok() && count<=40){
        cap.read(img);
        // imshow("AAAAAA", img);
        loop_rate.sleep();
        count++;
    }

    imwrite(PATH, img);
    img.release();
}

void VISION::E_image(void){
    const double epsilon = 5;  // DP Algorithm 的參數
    const int minContour = 3;  // 邊數小於 minContour 會被遮罩
    const int maxContour = 6;  // 邊數大於 maxContour 
    const double lowerBondArea = 10;  // 面積低於 lowerBondArea 的輪廓會被遮罩
    
    std::string path = PATH;
    Mat src = imread(PATH);

    // resize(src, src, Size(src.cols/2, src.rows/2));
    Mat original_image = src.clone();
    
    src = VISION::E_filter(src);
    VISION::E_contour(original_image, src, epsilon, minContour, maxContour, lowerBondArea);
}


void VISION::CTFL_image(void){
/// 需要調整的變數
    const double epsilon = 3;  // DP Algorithm 的參數
    const int minContour = 4;  // 邊數小於 minContour 會被遮罩
    const int maxContour = 9;  // 邊數大於 maxContour 會遮罩
    const double lowerBondArea = 20;  // 面積低於 lowerBondArea 的輪廓會被遮罩
///             

    std::string path = PATH;
    Mat src = imread(PATH);

    // resize(src, src, Size(src.cols, src.rows));
    Mat original_image = src.clone();
    // imshow("original",original_image);
    
    src = VISION::CTFL_filter(src);  
    VISION::CTFL_contour(original_image, src, epsilon, minContour, maxContour, lowerBondArea);    
}


/** Internal Functioin **/
Mat VISION::E_filter(Mat img){
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

void VISION::E_contour(Mat original_image, Mat image, double epsilon, \
    int minContour, int maxContour, double lowerBondArea){
    cvtColor(image, image, COLOR_BGR2GRAY);
    threshold(image, image, 40, 255, THRESH_BINARY);

    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;

// 1) 找出邊緣
    findContours(image, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    // imshow("A", image);

    std::vector<std::vector<Point>> polyContours(contours.size());  // polyContours 用來存放折線點的集合


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
    std::vector<std::vector<Point>> contours2;
    std::vector<Vec4i> hierarchy2;

    findContours(dp_optim_v1_image, contours2, hierarchy2, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);


// 6) 簡化好輪廓 DP演算法
    std::vector<std::vector<Point>> polyContours2(contours2.size());  // 存放折線點的集合
    Mat dp_image_2 = Mat::zeros(dp_optim_v1_image.size(), CV_8UC3);
    for(size_t i=0; i < contours2.size(); i++){
        approxPolyDP(Mat(contours2[i]), polyContours2[i], epsilon, true);
    }
    drawContours(dp_image_2, polyContours2, -2, Scalar(255,0,255), 1, 0);


// 7) 擬和旋轉矩形 + 標示方塊中心點
    RotatedRect box;  // 旋轉矩形 class
    Point2f vertices[4];  // 旋轉矩形四頂點
    std::vector<Point> pt;  // 存一個contour中的點集合


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


        if( !E_isDetected ){
            E_isDetected = true;
            VISION::detect[1]=(vertices[0]+vertices[1]+vertices[2]+vertices[3])/4;

            std::cout<<"\n-- Point E --\nX: "<<(vertices[0].x+vertices[1].x+vertices[2].x+vertices[3].x)/4 \
            <<"\nY: "<<(vertices[0].y+vertices[1].y+vertices[2].y+vertices[3].y)/4<<std::endl;
        }

    }
    // imshow("D", dp_image_2);
    // imshow("E", original_image);
    // waitKey(0);
}


Mat VISION::CTFL_filter(Mat img){
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

void VISION::CTFL_contour(Mat original_image, Mat image, double epsilon, \
    int minContour, int maxContour, double lowerBondArea){
    cvtColor(image, image, COLOR_BGR2GRAY);
    threshold(image, image, 40, 255, THRESH_BINARY);

    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;

// 1) 找出邊緣
    findContours(image, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    // imshow("Contours Image (before DP)", image);

    std::vector<std::vector<Point>> polyContours(contours.size());  // polyContours 用來存放折線點的集合

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
    std::vector<std::vector<Point>> contours2;
    std::vector<Vec4i> hierarchy2;

    findContours(dp_optim_v1_image, contours2, hierarchy2, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

// 5) 簡化好輪廓 DP演算法
    std::vector<std::vector<Point>> polyContours2(contours2.size());  // 存放折線點的集合
    Mat dp_image_2 = Mat::zeros(dp_optim_v1_image.size(), CV_8UC3);
    for(size_t i=0; i < contours2.size(); i++){
        approxPolyDP(Mat(contours2[i]), polyContours2[i], epsilon, true);
    }
    drawContours(dp_image_2, polyContours2, -2, Scalar(255,0,255), 1, 0);

    Mat dp_image_text = dp_image_2.clone();
    // imshow("Contours Image (After DP):", dp_image_text);


// 7) 擬和旋轉矩形 + 邊長數量判斷字型 + 標示方塊中心點
    RotatedRect box;  // 旋轉矩形 class
    Point2f vertices[4];  // 旋轉矩形四頂點
    std::vector<Point> pt;  // 存一個contour中的點集合

    for(int a=0; a<polyContours2.size(); a++){
    // A) 旋轉矩形
        pt.clear();
        for(int b=0; b<polyContours2[a].size(); b++){
            pt.push_back(polyContours2[a][b]);
        }
        box = minAreaRect(pt);  // 找到最小矩形，存到 box 中
        box.points(vertices);  // 把矩形的四個頂點資訊丟給 verti    ces，points()是 RotatedRect 的函式

        for(int i=0; i<4; i++){
            line(dp_image_2, vertices[i], vertices[(i+1)%4], Scalar(0,255,0), 2);  // 描出旋轉矩形
        }

        // 標示
        circle(dp_image_2, (vertices[0]+vertices[1]+vertices[2]+vertices[3])/4, 0, Scalar(0,255,255), 8);  // 繪製中心點
        circle(original_image, (vertices[0]+vertices[1]+vertices[2]+vertices[3])/4, 0, Scalar(0,255,255), 8);  // 與原圖比較
    
    // B) 判斷字母(用邊長個數篩選)
        if(polyContours2[a].size() == 6 || polyContours2[a].size() == 7){  // L 
            // 標示
            putText(dp_image_2, "L", (vertices[0]+vertices[1]+vertices[2]+vertices[3])/4, 1, 3, Scalar(0, 255, 255), 3);
            putText(original_image, "L", (vertices[0]+vertices[1]+vertices[2]+vertices[3])/4, 1, 3, Scalar(0, 0, 255), 2);
            
            if( L_isDetected==false ){
                L_isDetected=true;
                VISION::detect[2]=(vertices[0]+vertices[1]+vertices[2]+vertices[3])/4;  
                
                std::cout<<"\n-- Point L --\nX: "<<(vertices[0].x+vertices[1].x+vertices[2].x+vertices[3].x)/4 \
                <<"\nY: "<<(vertices[0].y+vertices[1].y+vertices[2].y+vertices[3].y)/4<<std::endl;
            }
        }

        if(polyContours2[a].size() == 8 || polyContours2[a].size() == 9){  // T、E (此時場上不會有 E)
            // 標示
            putText(dp_image_2, "T", (vertices[0]+vertices[1]+vertices[2]+vertices[3])/4, 1, 3, Scalar(0, 255, 255), 3);
            putText(original_image, "T", (vertices[0]+vertices[1]+vertices[2]+vertices[3])/4, 1, 3, Scalar(0, 0, 255),2);
            
            if( T_isDetected==false ){
                T_isDetected = true;
                VISION::detect[0]=(vertices[0]+vertices[1]+vertices[2]+vertices[3])/4;

                std::cout<<"\n-- Point T --\nX: "<<(vertices[0].x+vertices[1].x+vertices[2].x+vertices[3].x)/4 \
                <<"\nY: "<<(vertices[0].y+vertices[1].y+vertices[2].y+vertices[3].y)/4<<std::endl;
            }
        }

    }

    // imshow("Contours Filted", dp_image_2);
    // imshow("Original Image (highlight)", original_image);
    // waitKey(0);
}
