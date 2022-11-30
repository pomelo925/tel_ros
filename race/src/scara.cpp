#include "race/scara.h"
#include "race/vision.h"

namespace SCARA{
    double vision_x, vision_y, scaraflag;
}

void SCARA::init(void){
    ros::NodeHandle nh_4scara;
    scara_pub = nh_4scara.advertise<geometry_msgs::Point>("scara_toSTM", 1);
    scara_sub = nh_4scara.subscribe("scaraflag_fromSTM", 1, SCARA::callback);

    nh_4scara.getParam("SCARA_MODE", SCARA::MODE);
}

void SCARA::callback(const std_msgs::Float64::ConstPtr &flag){
    scaraflag = flag->data;
}

void SCARA::movingTo(double x, double y, double z){
    while(scaraflag!=0 && ros::ok()) ros::spinOnce();

    while(scaraflag==0 && ros::ok()){
        scara_cor.x = x;
        scara_cor.y = y;
        scara_cor.z = z;
        scara_pub.publish(scara_cor);
        ros::spinOnce();
    }
}

void SCARA::tel_1(void){
    /* 零點歸位*/
    printf("    SCARA::movingTo(0, -20, 1) \n");
    SCARA::movingTo(0, -50, 1);
    
    /* 定點拍照*/
    printf("    SCARA::movingTo(-330, 0, 2) \n");
    SCARA::movingTo(-330, 0, 2);
    VISION::taking_photo();
    
    /* 辨識*/
    VISION::E_image();
    // SCARA::seize();


    /* 定點拍照*/
    printf("    SCARA::movingTo(-330, 0, 2) \n");
    SCARA::movingTo(-330, 0, 2);
    VISION::taking_photo();
    
    /* 辨識*/
    VISION::CTFL_image();
    // SCARA::seize();
}


void SCARA::tel_2(void){
    SCARA::movingTo(-330, 0, 2);

    /* 定點拍照 */
    VISION::taking_photo();

    /* 辨識 */
    VISION::E_image();
    VISION::CTFL_image();

    /* 抓 */
    // SCARA::seize();
}


void SCARA::cubeoff(void){
    SCARA::movingTo(0, -330, 4); 
    
    SCARA::movingTo(0, -50, 2);
    SCARA::movingTo(0, -50, 5);
}

void SCARA::seize(void){
    Point2f priority[3];

    /* 判斷是否為「新」辨識出的方塊*/
    if(VISION::T_isDetected && !VISION::T_isCatched){
        priority[0]=VISION::detect[0];
        VISION::T_isCatched = true;
    }
    else priority[0].y = 9999.99;

    if(VISION::E_isDetected && !VISION::E_isCatched){
        priority[1]=VISION::detect[1];
        VISION::E_isCatched = true;   
    }
    else priority[1].y = 9999.99;

    if(VISION::L_isDetected && !VISION::L_isCatched){
        priority[2]=VISION::detect[2];
        VISION::L_isCatched = true;
    }
    else priority[2].y = 9999.99;

    /* 將 tf 過後的point 以 y 小到大排列*/
    qsort(priority, 3, sizeof(Point2f), SCARA::compare);

    for(int i=0; i<3; i++){
        printf("\npriority[%d]: %f, %f\n", i, priority[i].x, priority[i].y);
    }

    /* 配合 SCARA 左->右->中*/
    if (priority[0].y > 1000) goto clean;
    else{
        SCARA::movingTo(priority[0].x*10, priority[0].y*10, 3); printf("%lf\t%lf\nseize A finish\n",priority[0].x*10, priority[0].y*10);
    }
    
    if (priority[2].y < 1000) {
        SCARA::movingTo(priority[2].x*10, priority[2].y*10, 3); printf("\nseize B finish\n");
        SCARA::movingTo(priority[1].x*10, priority[1].y*10, 3); printf("\nseize C finish\n");
    } else {
        if(priority[1].y < 1000){
            SCARA::movingTo(priority[1].x*10, priority[1].y*10, 3); printf("\nseize B finish\n");
        }
        else goto clean;
    }

    clean:{
        for(int i=0; i<3; i++) priority[i].x=0, priority[i].y=0;
        return;
    }
}


int SCARA::compare(const void *a, const void *b){
	Point2f* m = (Point2f*) a;
	Point2f* n = (Point2f*) b;

    float la = m->y;
    float lb = n->y;
	
    if (la > lb) return 1;
	else if (la < lb) return -1;

	return 0;
}