#ifndef _SCARA_H_
#define _SCARA_H_

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Point.h"

std_msgs::Float64 status;
ros::Subscriber scara_sub;

ros::Publisher scara_pub;
geometry_msgs::Point scara_cor;


namespace SCARA{
    bool MODE = true; // mode 0 : dont do vision
    extern double vision_x, vision_y, scaraflag;

    void init(void);
    void movingTo(double x, double y, double z);
    void tel_1(void);
    void tel_2(void);
    void cubeoff(void);
    void seize(void);
    int compare(const void *a, const void *b);

    void callback(const std_msgs::Float64::ConstPtr &flag);
}

#endif 