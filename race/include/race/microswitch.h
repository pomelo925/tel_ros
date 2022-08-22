#ifndef _MICROSWITCH_H_
#define _MICROSWITCH_H_

#include <ros/ros.h>
#include <std_msgs/Int64MultiArray.h>

#define TRUE 1
#define FALSE 0

ros::Subscriber chassis_switch_subscriber;  // topic: chassis_switch_fromSTM
std_msgs::Int64MultiArray chassis_switch_sub;
void switch_callback(const std_msgs::Int64MultiArray::ConstPtr &switch_msg);

void switch_init();

class CH_MICRO{
    private:
        int A, B, C, D;
    
    public:
        CH_MICRO(int a, int b, int c, int d);
        bool isTouch();
};



#endif