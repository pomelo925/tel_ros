#include "race/mecanum.h"

// declare NodeHandle and pub/sub
void MECANUM::init(){
    ros::NodeHandle nh_4mecanum;
    mecanum_publisher = nh_4mecanum.advertise<geometry_msgs::Point>("mecanum_toSTM", 1);
    mecanum_subscriber = nh_4mecanum.subscribe("mecanum_fromSTM", 1, MECANUM::callback);
    
    // odom_pub = nh_4mecanum.advertise<nav_msgs::Odometry>("odom", 1);
}

// encdoer callback function and publish
void MECANUM::callback(const geometry_msgs::Point::ConstPtr& vel){
    mecanum_sub.x = vel->x;
    mecanum_sub.y = vel->y;
    mecanum_sub.z = vel->z;

    data_check = true;
} 


void MECANUM::moveTo(double x_cor, double y_cor, double z_cor){
    z_cor*=3.1415926/180;
    double acc_x=0, acc_y=0, acc_z=0;
    double x_now=0, y_now=0, z_now=0;
    double x_err = x_cor, y_err = y_cor, z_err=z_cor;  // distance between now & goal
    double time_now, time_before;  
    double x_vel_before, y_vel_before, z_vel_before;  // velocity of previous instance
    bool flag = false;  // flag for NOT integral on first instance

    while( fabs(x_err) > x_tol_margin || fabs(y_err) > y_tol_margin || fabs(z_err) > z_tol_margin){
        // calculate error and pub new speed
        x_err = x_cor - x_now, y_err = y_cor - y_now, z_err = z_cor - z_now;

        /* velocity profile */
        mecanum_pub.x =0;
        mecanum_pub.y =0;
        mecanum_pub.z =0;

        /// accerlation ///
        if( fabs(x_err) > fabs(dr*x_cor) && fabs(x_err) > x_tol_margin){
            mecanum_pub.x=acc_x;
            acc_x += (x_err>0)? 0.05 : -0.05;
            if(acc_x >= 5) mecanum_pub.x=5;
            if(acc_x <=-5) mecanum_pub.x=-5;
        }
        if( fabs(y_err) > fabs(dr*y_cor) && fabs(y_err) > y_tol_margin){
            mecanum_pub.y=acc_y;
            acc_y += (y_err>0)? 0.05 : -0.05;
            if(acc_y >= 5) mecanum_pub.y=5;
            if(acc_y <=-5) mecanum_pub.y=-5;
        }
        if( fabs(z_err) > fabs(dr*z_cor) && fabs(z_err) > z_tol_margin){
            mecanum_pub.z=acc_z;
            acc_z += (z_err>0)? 0.001 : -0.001;
            if(acc_z >= 0.15) mecanum_pub.z=0.15;
            if(acc_z <= -0.15) mecanum_pub.z=-0.15;
        }

        /// deceleration ///

        // if( fabs(x_err) <= fabs(1/3.0*x_cor) && x_cor!=0) mecanum_pub.x*=kp_xy*(fabs(x_err/x_cor));
        // if( fabs(y_err) <= fabs(1/3.0*y_cor) && y_cor!=0) mecanum_pub.y*=kp_xy*(fabs(y_err/y_cor));
        // if( fabs(z_err) <= fabs(1/3.0*z_cor) && z_cor!=0) mecanum_pub.z*=kp_z*(fabs(z_err/z_cor));

        if( fabs(x_err) <= fabs(dr*x_cor) && x_cor!=0) mecanum_pub.x = kp_xy*x_err;
        if( fabs(y_err) <= fabs(dr*y_cor) && y_cor!=0) mecanum_pub.y = kp_xy*y_err;
        if( fabs(z_err) <= fabs(dr*z_cor) && z_cor!=0) mecanum_pub.z = kp_z*z_err;
        mecanum_publisher.publish(mecanum_pub);

        /* velocity profile */
        
        // MECANUM::TF(x_now, y_now, z_now, ros::Time::now());
        data_check = false;
        while(!data_check) ros::spinOnce();

        // integral (unit: cm/s) 
        time_now = ros::Time::now().toSec();
 
        if(flag){
            x_now += (time_now - time_before) * (mecanum_sub.x + x_vel_before)/2;
            y_now += ( (time_now - time_before) * (mecanum_sub.y + y_vel_before)/2 ); 
            z_now += ( (time_now - time_before) * (mecanum_sub.z + z_vel_before)/2 ); 
        }   flag = true;
        
        std::cout<<"X: "<<x_now<<"\t\tVx: "<<mecanum_pub.x<<std::endl;
        std::cout<<"Y: "<<y_now<<"\t\tVy: "<<mecanum_pub.y<<std::endl;
        std::cout<<"Z: "<<z_now<<"\t\tVz: "<<mecanum_pub.z<<std::endl;
        std::cout<<"= = = = = = = = ="<<std::endl;

        x_vel_before = mecanum_sub.x;
        y_vel_before = mecanum_sub.y;
        z_vel_before = mecanum_sub.z;
        time_before = time_now;
        ros::Duration(0.01).sleep();
    }
    
    // reaching goal and pub speed 0
    while(mecanum_sub.x != 0 || mecanum_sub.y != 0 || mecanum_sub.z != 0){
        mecanum_pub.x = 0;
        mecanum_pub.y = 0;
        mecanum_pub.z = 0;
        mecanum_publisher.publish(mecanum_pub);

        data_check = false;
        while(!data_check) ros::spinOnce();
        

        std::cout<<"X: "<<x_now<<"\t\tVx: "<<mecanum_pub.x<<std::endl;
        std::cout<<"Y: "<<y_now<<"\t\tVy: "<<mecanum_pub.y<<std::endl;
        std::cout<<"Z: "<<z_now<<"\t\tVz: "<<mecanum_pub.z<<std::endl;
        std::cout<<"+ + + about to stop + + +"<<std::endl;
    }

    // MECANUM::TF(x_now, y_now, z_now, ros::Time::now());
}

/*** MoveTo -- Overloading TYPE 5***/
// moveTo(POINT) : integration of TYPE 1 & 3
void MECANUM::moveTo(POINT point){
    double acc_x=0, acc_y=0, acc_z=0;
    double x_now=0, y_now=0, z_now=0;
    double x_err = point.x_cor, y_err = point.y_cor, z_err=point.z_cor*3.1415926/180;  // distance between now & goal
    double time_now, time_before;  
    double x_vel_before, y_vel_before, z_vel_before;  // velocity of previous instance
    bool flag = false;  // flag for NOT integral on first instance

   while( fabs(x_err) > x_tol_margin || fabs(y_err) > y_tol_margin || fabs(z_err) > z_tol_margin){
        // calculate error and pub new speed
        x_err = point.x_cor - x_now, y_err = point.y_cor - y_now, z_err = point.z_cor - z_now;

        /* velocity profile */
        mecanum_pub.x =0;
        mecanum_pub.y =0;
        mecanum_pub.z =0;

        /// accerlation ///
        if( fabs(x_err) > fabs(dr* point.x_cor) && fabs(x_err) > x_tol_margin){
            mecanum_pub.x=acc_x;
            acc_x += (x_err>0)? 0.01 : -0.01;
            if(acc_x >= 5) mecanum_pub.x=5;
            if(acc_x <=-5) mecanum_pub.x=-5;
        }
        if( fabs(y_err) > fabs(dr* point.y_cor) && fabs(y_err) > y_tol_margin){
            mecanum_pub.y= acc_y;
            acc_y += (y_err>0)? 0.01 : -0.01;
            if(acc_y >= 5) mecanum_pub.y=5;
            if(acc_y <=-5) mecanum_pub.y=-5;
        }
        if( fabs(z_err) > fabs(dr* point.z_cor) && fabs(z_err) > z_tol_margin){
            mecanum_pub.z=acc_z;
            acc_z += (z_err>0)? 0.001 : -0.001;
            if(acc_z >= 0.15) mecanum_pub.z=0.15;
            if(acc_z <= -0.15) mecanum_pub.z=-0.15;
        }

        /// deceleration ///

        // if( fabs(x_err) <= fabs(1/3.0*x_cor) && x_cor!=0) mecanum_pub.x*=kp_xy*(fabs(x_err/x_cor));
        // if( fabs(y_err) <= fabs(1/3.0*y_cor) && y_cor!=0) mecanum_pub.y*=kp_xy*(fabs(y_err/y_cor));
        // if( fabs(z_err) <= fabs(1/3.0*z_cor) && z_cor!=0) mecanum_pub.z*=kp_z*(fabs(z_err/z_cor));

        if( fabs(x_err) <= fabs(dr*point.x_cor) && point.x_cor!=0) mecanum_pub.x = kp_xy*x_err;
        if( fabs(y_err) <= fabs(dr*point.y_cor) && point.y_cor!=0) mecanum_pub.y = kp_xy*y_err;
        if( fabs(z_err) <= fabs(dr*point.z_cor) && point.z_cor!=0) mecanum_pub.z = kp_z*z_err;
        mecanum_publisher.publish(mecanum_pub);

        /* velocity profile */
        
        // MECANUM::TF(x_now, y_now, z_now, ros::Time::now());
        data_check = false;
        while(!data_check) ros::spinOnce();

        // integral (unit: cm/s) 
        time_now = ros::Time::now().toSec();
        // WARNING!!!  STRANGE COEFFICIENT BUG !!!
        if(flag){
            x_now += (time_now - time_before) * (mecanum_sub.x + x_vel_before)/2;
            y_now += 5.4*( (time_now - time_before) * (mecanum_sub.y + y_vel_before)/2 ); 
            z_now += 4*( (time_now - time_before) * (mecanum_sub.z + z_vel_before)/2 ); 
        }   flag = true;
        
        std::cout<<"X: "<<x_now<<"\t\tVx: "<<mecanum_pub.x<<std::endl;
        std::cout<<"Y: "<<y_now<<"\t\tVy: "<<mecanum_pub.y<<std::endl;
        std::cout<<"Z: "<<z_now<<"\t\tVz: "<<mecanum_pub.z<<std::endl;
        std::cout<<"= = = = = = = = ="<<std::endl;

        x_vel_before = mecanum_sub.x;
        y_vel_before = mecanum_sub.y;
        z_vel_before = mecanum_sub.z;
        time_before = time_now;
        ros::Duration(0.01).sleep();
    }
    
    // reaching goal and pub speed 0
    while(mecanum_sub.x != 0 || mecanum_sub.y != 0 || mecanum_sub.z != 0){
        mecanum_pub.x = 0;
        mecanum_pub.y = 0;
        mecanum_pub.z = 0;
        mecanum_publisher.publish(mecanum_pub);

        data_check = false;
        while(!data_check) ros::spinOnce();
        

        std::cout<<"X: "<<x_now<<"\t\tVx: "<<mecanum_pub.x<<std::endl;
        std::cout<<"Y: "<<y_now<<"\t\tVy: "<<mecanum_pub.y<<std::endl;
        std::cout<<"Z: "<<z_now<<"\t\tVz: "<<mecanum_pub.z<<std::endl;
        std::cout<<"+ + + about to stop + + +"<<std::endl;
    }

    // MECANUM::TF(x_now, y_now, z_now, ros::Time::now());
}

void MECANUM::TF(double x, double y, double z, ros::Time time_now){  
    static tf::TransformBroadcaster odom_broadcaster;

//first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = time_now;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    double vx = mecanum_sub.x;
    double vy = mecanum_sub.y;
    double vz = mecanum_sub.z;

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