#ifndef _MICROSWITCH_H_
#define _MICROSWITCH_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#define TRUE 1
#define FALSE 0

ros::Subscriber microswitch_sub;  // topic: chassis_switch_fromSTM
geometry_msgs::Point chassis_switch_sub;

namespace SWITCH{
    void callback(const geometry_msgs::Point::ConstPtr &switch_msg);
    void init();
}

class CH_MICRO{
    public:
        int fr, fl, lf;
    
        CH_MICRO(int fr, int fl, int lf);
        bool isTouch();
};



#endif