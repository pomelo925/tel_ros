#include "race/encoder.h"

// declare NodeHandle and pub/sub
void ENCODER::init(){
    ros::NodeHandle nh_4encoder;
    encoder_publisher = nh_4encoder.advertise<geometry_msgs::Point>("encoder_toSTM", 1);
    encoder_subscriber = nh_4encoder.subscribe("encoder_fromSTM", 1, ENCODER::callback);
}

// encdoer callback function
void ENCODER::callback(const geometry_msgs::Point::ConstPtr& vel){
    encoder_sub.x = vel->x;
    encoder_sub.y = vel->y;
    encoder_sub.z = vel->z;
} 


/*** MoveTo -- Overloading TYPE 3***/
// moveTo(X,Y,Z) : XYZ is RELATIVE coordinate to current one.
void ENCODER::moveTo(double x_cor, double y_cor, double z_cor){
    double x_now=0, y_now=0, z_now;  // // current car coordinate
    double x_err = x_cor, y_err = y_cor, z_err=z_cor;  // distance between now & goal
    double time_now, time_before;  
    double x_vel_before, y_vel_before, z_vel_before;  // velocity of previous instance
    bool flag = false;  // flag for NOT integral on first instance

    while( x_err > x_tol_margin && y_err > y_tol_margin && z_err > z_tol_margin){
        // calculate error and pub new speed
        x_err = x_cor - x_now, y_err = y_cor - y_now, z_err = z_cor - z_now;

        encoder_pub.x = x_err*p_coe;
        encoder_pub.y = y_err*p_coe;
        encoder_pub.z = z_err*p_coe;
        encoder_publisher.publish(encoder_pub);

        // read encoder data
        ros::spinOnce();
        time_now = ros::Time::now().toSec();

        // integral (unit: cm/s) 
        if(flag){
            x_now += (time_now - time_before) * (encoder_sub.x + x_vel_before)/2;
            y_now += (time_now - time_before) * (encoder_sub.y + y_vel_before)/2; 
            z_now += (time_now - time_before) * (encoder_sub.z + z_vel_before)/2; 
        }   flag = true;
        
        x_vel_before = encoder_sub.x;
        y_vel_before = encoder_sub.y;
        z_vel_before = encoder_sub.z;
        time_before = time_now;

    }
    
    // reaching goal and pub speed 0
        encoder_pub.x = 0;
        encoder_pub.y = 0;
        encoder_publisher.publish(encoder_pub);
}


/*** MoveTo -- Overloading TYPE 4***/
// moveTo(X,Y,Z, MICRO) : XYZ is RELATIVE coordinate to current one.
// MICRO considers the trigger of microswitch.
void ENCODER::moveTo(double x_cor, double y_cor, double z_cor, CH_MICRO condition){
    double x_now=0, y_now=0, z_now;  // // current car coordinate
    double x_err = x_cor, y_err = y_cor, z_err=z_cor;  // distance between now & goal
    double time_now, time_before;  
    double x_vel_before, y_vel_before, z_vel_before;  // velocity of previous instance
    bool flag = false;  // flag for NOT integral on first instance

    while( x_err > x_tol_margin && y_err > y_tol_margin && z_err > z_tol_margin && condition.isTouch() == false){
        // calculate error and pub new speed
        x_err = x_cor - x_now, y_err = y_cor - y_now, z_err = z_cor - z_now;

        encoder_pub.x = x_err*p_coe;
        encoder_pub.y = y_err*p_coe;
        encoder_pub.z = z_err*p_coe;
        encoder_publisher.publish(encoder_pub);

        // read encoder data
        ros::spinOnce();
        time_now = ros::Time::now().toSec();

        // integral (unit: cm/s)  
        if(flag){
            x_now += (time_now - time_before) * (encoder_sub.x + x_vel_before)/2;
            y_now += (time_now - time_before) * (encoder_sub.y + y_vel_before)/2; 
            z_now += (time_now - time_before) * (encoder_sub.z + z_vel_before)/2; 
        }   flag = true;
        

        x_vel_before = encoder_sub.x;
        y_vel_before = encoder_sub.y;
        z_vel_before = encoder_sub.z;
        time_before = time_now;
    
        // break once microswitch triggered
        if(condition.isTouch() == true) break;
    }
    
    // reaching goal and pub speed 0
        encoder_pub.x = 0;
        encoder_pub.y = 0;
        encoder_publisher.publish(encoder_pub);
}


/*** MoveTo -- Overloading TYPE 5***/
// moveTo(POINT) : integration of TYPE 1 & 3
void ENCODER::moveTo(POINT point){
    double x_now=0, y_now=0, z_now;  // // current car coordinate
    double x_err = point.x_cor, y_err = point.y_cor, z_err=point.z_cor;  // distance between now & goal
    double time_now, time_before;  
    double x_vel_before, y_vel_before, z_vel_before;  // velocity of previous instance
    bool flag = false;  // flag for NOT integral on first instance

    while( x_err > x_tol_margin && y_err > y_tol_margin && z_err > z_tol_margin){
        // calculate error and pub new speed
        x_err = point.x_cor - x_now, y_err = point.y_cor - y_now, z_err = point.z_cor - z_now;

        encoder_pub.x = x_err*p_coe;
        encoder_pub.y = y_err*p_coe;
        encoder_pub.z = z_err*p_coe;
        encoder_publisher.publish(encoder_pub);

        // read encoder data
        ros::spinOnce();
        time_now = ros::Time::now().toSec();

        // integral (unit: cm/s) 
        if(flag){
            x_now += (time_now - time_before) * (encoder_sub.x + x_vel_before)/2;
            y_now += (time_now - time_before) * (encoder_sub.y + y_vel_before)/2; 
            z_now += (time_now - time_before) * (encoder_sub.z + z_vel_before)/2; 
        }   flag = true;
        
        x_vel_before = encoder_sub.x;
        y_vel_before = encoder_sub.y;
        z_vel_before = encoder_sub.z;
        time_before = time_now;

    }
    
    // reaching goal and pub speed 0
        encoder_pub.x = 0;
        encoder_pub.y = 0;
        encoder_publisher.publish(encoder_pub);
}


/*** MoveTo -- Overloading TYPE 6***/
// moveTo(POINT) : integration of TYPE 2 & 4
void ENCODER::moveTo(POINT point, CH_MICRO condition){
    double x_now=0, y_now=0, z_now;  // // current car coordinate
    double x_err = point.x_cor, y_err = point.y_cor, z_err=point.z_cor;  // distance between now & goal
    double time_now, time_before;  
    double x_vel_before, y_vel_before, z_vel_before;  // velocity of previous instance
    bool flag = false;  // flag for NOT integral on first instance

    while( x_err > x_tol_margin && y_err > y_tol_margin && z_err > z_tol_margin && condition.isTouch() == false){
        // calculate error and pub new speed
        x_err = point.x_cor - x_now, y_err = point.y_cor - y_now, z_err = point.z_cor - z_now;

        encoder_pub.x = x_err*p_coe;
        encoder_pub.y = y_err*p_coe;
        encoder_pub.z = z_err*p_coe;
        encoder_publisher.publish(encoder_pub);

        // read encoder data
        ros::spinOnce();
        time_now = ros::Time::now().toSec();

        // integral (unit: cm/s)  
        if(flag){
            x_now += (time_now - time_before) * (encoder_sub.x + x_vel_before)/2;
            y_now += (time_now - time_before) * (encoder_sub.y + y_vel_before)/2; 
            z_now += (time_now - time_before) * (encoder_sub.z + z_vel_before)/2; 
        }   flag = true;
        

        x_vel_before = encoder_sub.x;
        y_vel_before = encoder_sub.y;
        z_vel_before = encoder_sub.z;
        time_before = time_now;
    
        // break once microswitch triggered
        if(condition.isTouch() == true) break;
    }
    
    // reaching goal and pub speed 0
        encoder_pub.x = 0;
        encoder_pub.y = 0;
        encoder_publisher.publish(encoder_pub);
}
