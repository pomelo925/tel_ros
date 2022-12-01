#ifndef _RESET_H_
#define _RESET_H_

#include "ros/ros.h"
#include <std_msgs/Int64.h>
#include <iostream>

ros::Subscriber reset_sub;
void reset_callback(const std_msgs::Int64::ConstPtr& reset_data);

namespace RESET{
    int state = 0;
    std::string command = "rosparam set reset_state ";
}


#endif