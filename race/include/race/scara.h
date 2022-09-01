#ifndef _SCARA_H_
#define _SCARA_H_

#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "geometry_msgs/Point.h"

ros::Publisher putter_pub;
std_msgs::Int64 status;

ros::Publisher scara_pub;
geometry_msgs::Point scara_cor;

namespace PUTTER{
    void expansion(bool open);
}

namespace SCARA{
    void init(void);
    void catching(double x, double y);
}

#endif 