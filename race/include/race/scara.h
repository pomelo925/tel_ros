#ifndef _SCARA_H_
#define _SCARA_H_

#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "geometry_msgs/Point.h"

std_msgs::Int64 status;

ros::Publisher scara_pub;
geometry_msgs::Point scara_cor;


namespace SCARA{
    void init(void);
    void movingTo(double x, double y);
}

#endif 