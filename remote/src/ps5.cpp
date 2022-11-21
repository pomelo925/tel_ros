#include <ros/ros.h>
#include <geometry_msgs/Point.h>

using namespace std;

void mecanum_callback(const geometry_msgs::Point::ConstPtr& mecanum_msg);
void intake_callback(const geometry_msgs::Point::ConstPtr& intake_msg);

geometry_msgs::Point remote_mecanum_msgs;
geometry_msgs::Point remote_intake_msgs;

int main(int argc, char **argv){
    ros::init(argc, argv, "ps5");
    ros::NodeHandle nh_ps5;

    ros::Publisher remote_mecanum_pub = nh_ps5.advertise<geometry_msgs::Point>("mecanum_toSTM", 1);
    ros::Publisher remote_intake_pub = nh_ps5.advertise<geometry_msgs::Point>("intake_toSTM", 1);
    ros::Subscriber remote_mecanum_sub = nh_ps5.subscribe("mecanum_fromArduino",1, mecanum_callback);
    ros::Subscriber remote_intake_sub = nh_ps5.subscribe("intake_fromArduino",1, intake_callback);

    while(ros::ok()){
        ros::spinOnce();
        remote_mecanum_pub.publish(remote_mecanum_msgs);
        remote_intake_pub.publish(remote_intake_msgs); 
        ros::Duration(0.01).sleep();      
    }
}

void mecanum_callback(const geometry_msgs::Point::ConstPtr& mecanum_msgs){
    remote_mecanum_msgs.x = mecanum_msgs->x;
    remote_mecanum_msgs.y = mecanum_msgs->y;
    remote_mecanum_msgs.z = mecanum_msgs->z;

    // cout<<"**** MECANUM_INFO ****"<<endl;
    // cout<<"X: "<<remote_mecanum_msgs.x<<endl;
    // cout<<"Y: "<<remote_mecanum_msgs.y<<endl;
    // cout<<"Z: "<<remote_mecanum_msgs.z<<endl;    
}

void intake_callback(const geometry_msgs::Point::ConstPtr& intake_msgs){
    remote_intake_msgs.x = intake_msgs->x;
    remote_intake_msgs.y = intake_msgs->y;
    remote_intake_msgs.z = intake_msgs->z;
    
    cout<<"**** INTAKE_INFO ****"<<endl;
    cout<<"X: "<<remote_intake_msgs.x<<endl;
    cout<<"Y: "<<remote_intake_msgs.y<<endl;
    cout<<"Z: "<<remote_intake_msgs.z<<endl;    
}