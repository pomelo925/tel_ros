#include "race/encoder.h"

// declare NodeHandle and pub/sub
void ENCODER::init(){
    ros::NodeHandle nh_4encoder;
    encoder_publisher = nh_4encoder.advertise<geometry_msgs::Point>("mecanum_toSTM", 1);
    encoder_subscriber = nh_4encoder.subscribe("mecanum_fromSTM", 1, ENCODER::callback);
    
    odom_pub = nh_4encoder.advertise<nav_msgs::Odometry>("odom", 1);
}

// encdoer callback function and publish
void ENCODER::callback(const geometry_msgs::Point::ConstPtr& vel){
    encoder_sub.x = vel->x;
    encoder_sub.y = vel->y;
    encoder_sub.z = vel->z;

    data_check = true;
} 


void ENCODER::moveTo(double x_cor, double y_cor, double z_cor){
    z_cor*=3.1415926/180;
    double acc_x=0, acc_y=0, acc_z=0;
    double x_now=0, y_now=0, z_now=0;
    double x_err = x_cor, y_err = y_cor, z_err=z_cor;  // distance between now & goal
    double time_now, time_before;  
    double x_vel_before, y_vel_before, z_vel_before;  // velocity of previous instance
    bool flag = false;  // flag for NOT integral on first instance

    while( x_err > x_tol_margin || y_err > y_tol_margin || z_err > z_tol_margin){
        // calculate error and pub new speed
        x_err = x_cor - x_now, y_err = y_cor - y_now, z_err = z_cor - z_now;

        /* velocity profile */
        encoder_pub.x = x_err*p_coe;
        encoder_pub.y = y_err*p_coe;
        encoder_pub.z = z_err*p_coe;

        if(x_err > 1/3*x_cor && encoder_pub.x!=0 && x_err > 0.01){
            encoder_pub.x=acc_x;
            acc_x += 0.5;
            if(acc_x >= 5) encoder_pub.x=5;
        }
        if(y_err > 1/3*y_cor && encoder_pub.y!=0 && y_err > 0.01){
            encoder_pub.y=acc_y;
            acc_y += 0.5;
            if(acc_y >= 5) encoder_pub.y=5;
        }
        if(z_err > 1/3*z_cor && encoder_pub.z!=0 && z_err > 0.01){
            encoder_pub.z=acc_z;
            acc_z += 0.2;
            if(acc_z >= 2) encoder_pub.z=2;
        }

        /* velocity profile */
        encoder_publisher.publish(encoder_pub);
        // ENCODER::TF(x_now, y_now, z_now, ros::Time::now());

        // read encoder data
        data_check = false;
        while(!data_check) ros::spinOnce();
        time_now = ros::Time::now().toSec();

        // integral (unit: cm/s) 
        if(flag){
            x_now += (time_now - time_before) * (encoder_sub.x + x_vel_before)/2;
            y_now += (time_now - time_before) * (encoder_sub.y + y_vel_before)/2; 
            z_now += (time_now - time_before) * (encoder_sub.z + z_vel_before)/2; 
        }   flag = true;
        
        std::cout<<"X: "<<x_now<<std::endl;
        std::cout<<"Y: "<<y_now<<std::endl;
        std::cout<<"Z: "<<z_now<<std::endl;

        x_vel_before = encoder_sub.x;
        y_vel_before = encoder_sub.y;
        z_vel_before = encoder_sub.z;
        time_before = time_now;
        ros::Duration(0.2).sleep();
    }
    
    // reaching goal and pub speed 0
    while(encoder_sub.x != 0 || encoder_sub.y != 0 || encoder_sub.z != 0){
        encoder_pub.x = 0.;
        encoder_pub.y = 0.;
        encoder_pub.z = 0.;
        ros::spinOnce();
    }

    ENCODER::TF(x_now, y_now, z_now, ros::Time::now());
}

/*** MoveTo -- Overloading TYPE 5***/
// moveTo(POINT) : integration of TYPE 1 & 3
void ENCODER::moveTo(POINT point){
    double x_now=0, y_now=0, z_now=0;
    double x_err = point.x_cor, y_err = point.y_cor, z_err=point.z_cor;  // distance between now & goal
    double time_now, time_before;  
    double x_vel_before, y_vel_before, z_vel_before;  // velocity of previous instance
    bool flag = false;  // flag for NOT integral on first instance

    while( x_err > x_tol_margin || y_err > y_tol_margin || z_err > z_tol_margin){
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

void ENCODER::TF(double x, double y, double z, ros::Time time_now){  
    static tf::TransformBroadcaster odom_broadcaster;

//first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = time_now;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    double vx = encoder_sub.x;
    double vy = encoder_sub.y;
    double vz = encoder_sub.z;

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(z);
    odom_trans.transform.rotation = odom_quat;

//send the transform
    odom_broadcaster.sendTransform(odom_trans);

// publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = time_now;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vz;

    //publish the message
    odom_pub.publish(odom);   
}