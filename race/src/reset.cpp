#include "race/reset.h"

void reset_callback(const std_msgs::Int64::ConstPtr& reset_data){
    RESET::state = reset_data->data;

    if(RESET::state!=0){
        RESET::command[25] = (char)RESET::state;
        system("rosnode kill /run"); 
        system( RESET::command );

        system("rosnode kill /reset"); 
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "reset");
    ros::NodeHandle nh_4reset;
    
    reset_sub = nh_4reset.subscribe("reset_fromSTM", 1, reset_callback);

    ros::spin();
}


