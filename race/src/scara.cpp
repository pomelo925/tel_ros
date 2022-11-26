#include "race/scara.h"
#include "race/vision.h"

namespace SCARA{
    double vision_x, vision_y, scaraflag;
}

void SCARA::init(void){
    ros::NodeHandle nh_4scara;
    scara_pub = nh_4scara.advertise<geometry_msgs::Point>("scara_toSTM", 1);
    scara_sub = nh_4scara.subscribe("scaraflag_fromSTM", 1, SCARA::callback);
}

void SCARA::callback(const std_msgs::Float64::ConstPtr &flag){
    scaraflag = flag->data;
}

void SCARA::movingTo(double x, double y, double z){
    scara_cor.x = x;
    scara_cor.y = y;
    scara_cor.z = z;
    scara_pub.publish(scara_cor);
    while(scaraflag!=0);
}

void SCARA::tel_1(void){
    /* 零點歸位*/
    SCARA::movingTo(0, -50, 1);
        ros::Duration(2).sleep(); while(scaraflag!=0.0); 
    
    /* 定點拍照*/
    SCARA::movingTo(-330, 0, 2);
        ros::Duration(2).sleep(); while(scaraflag!=0.0);    
    VISION::taking_photo();
    
    /* 辨識*/
    VISION::E_image();
    VISION::CTFL_image();

    /*抓*/
    SCARA::movingTo(-150, -220, 3);
        ros::Duration(2).sleep(); while(scaraflag!=0.0);
}


void SCARA::tel_2(void){
    SCARA::movingTo(-330, 0, 2);
        ros::Duration(2).sleep();
        while(scaraflag!=0.);

    VISION::taking_photo();
        ros::Duration(2).sleep();
        while(scaraflag!=0.);

    SCARA::movingTo(-200, 0, 3);
        ros::Duration(2).sleep();
        while(scaraflag!=0.);

    SCARA::movingTo(-180, -140, 3);
        ros::Duration(2).sleep();
        while(scaraflag!=0.);
}


void SCARA::cubeoff(void){
    SCARA::movingTo(0, 330, 4); 
        ros::Duration(3).sleep();
        while(scaraflag!=0.);
    
    SCARA::movingTo(0, -50, 2); 
        ros::Duration(3).sleep();
        while(scaraflag!=0.);
}

void SCARA::seize(void){
    Point2f priority[3];

    if(VISION::T_isDetected && !VISION::T_isCatched) priority[0]=VISION::detect[0];
    else priority[0].y = 999999;

    if(VISION::E_isDetected && !VISION::E_isCatched) priority[1]=VISION::detect[1];
    else priority[1].y = 999999;

    if(VISION::L_isDetected && !VISION::L_isCatched) priority[0]=VISION::detect[2];
    else priority[2].y = 999999;

    
    qsort(priority, 3, sizeof(Point2f), SCARA::compare);

    for(int i=0; i<3; i++){
        printf("priority[%d]: %f, %f\n", i, priority[i].x, priority[i].y);
    }

}

int SCARA::compare(const void *a, const void *b){
	Point2f* m = (Point2f*) a;
	Point2f* n = (Point2f*) b;

    float la = m->y;
    float lb = n->y;
	
    if (la > lb) return -1;
	else if (la < lb) return 1;

	return 0;
}