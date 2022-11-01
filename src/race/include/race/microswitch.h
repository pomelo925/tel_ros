#ifndef _MICROSWITCH_H_
#define _MICROSWITCH_H_

#include <ros/ros.h>
#include <std_msgs/Int64MultiArray.h>

#define TRUE 1
#define FALSE 0

ros::Subscriber chassis_switch_subscriber;  // topic: chassis_switch_fromSTM
std_msgs::Int64MultiArray chassis_switch_sub;

namespace SWITCH{
    void callback(const std_msgs::Int64MultiArray::ConstPtr &switch_msg);;
    void init();
}

class CH_MICRO{
    private:
        int fr, fl, lf, lb, bl, br;
    
    public:
        CH_MICRO(int fr, int fl, int lf, int lb, int bl, int br);
        bool isTouch();
};



#endif