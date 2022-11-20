#include "race/reset.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "reset");
    ros::NodeHandle nh;
    reset_sub = nh.subscribe("reset_fromSTM", 1, reset_callback);

    while(ros::ok()){
        ros::spinOnce();

        if(RESET::state!=0) system("rosnode kill /reset"); 
        ros::Duration(0.1).sleep();
    }
}

void reset_callback(const std_msgs::Int64::ConstPtr& reset_data){
    RESET::state = reset_data->data;

    RESET::command[25] = (char)RESET::state;
    system( RESET::command);
}