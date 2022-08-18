#include "race/velocity.h"
#include "race/microswitch.h"

// 初始化變數
void velocity_initial(){
    ros::NodeHandle nh_forVel;
    vel_publisher = nh_forVel.advertise<geometry_msgs::Point>("vel_toSTM", 1);
    vel_subscriber = nh_forVel.subscribe("vel_fromSTM", 1, vel_callback);
}

// 接受 encoder 速度的 callback function
void vel_callback(const geometry_msgs::Point::ConstPtr& vel){
} 


/*** MoveTo -- Overloading TYPE 1 ***/
// moveTo(X,Y)，XY 座標是「相對」於目前車身座標。
void moveTo(double x_cor, double y_cor){
    double x_now=0, y_now=0;  // 當前車身座標
    double x_tor = x_cor, y_tor = y_cor;  // 與目標差距
    double time_now, time_before;  
    double x_vel_before, y_vel_before;  // 上個瞬間的速度
    bool flag = false;  // 第一個瞬間不積分的flag

    while( x_tor > x_tol_margin && y_tor > y_tol_margin){
        // 計算誤差，publish 新速度
        x_tor = x_cor - x_now, y_tor = y_cor - y_now;

        vel_pub.x = x_tor*p_coe;
        vel_pub.y = y_tor*p_coe;
        vel_publisher.publish(vel_pub);

        // 接收 encoder
        ros::spinOnce();
        time_now = ros::Time::now().toSec();

        // 積分 (公分/秒) 
        if(flag){
            x_now += (time_now - time_before) * (vel_sub.x + x_vel_before)/2;
            y_now += (time_now - time_before) * (vel_sub.y + y_vel_before)/2; 
        }   flag = true;
        
        x_vel_before = vel_sub.x;
        y_vel_before = vel_sub.y;
        time_before = time_now;
    }

    // 到達目的地後靜止
        vel_pub.x = 0;
        vel_pub.y = 0;
        vel_publisher.publish(vel_pub);
}


/*** MoveTo -- Overloading TYPE 2 ***/
// moveTo(X,Y, MICRO)，XY 座標是「相對」於目前車身座標。
// MICRO 增加微動開關的判斷條件。
void moveTo(double x_cor, double y_cor, MICRO condition){
    double x_now=0, y_now=0;  // 當前車身座標
    double x_tor = x_cor, y_tor = y_cor;  // 與目標差距
    double time_now, time_before;  
    double x_vel_before, y_vel_before;  // 上個瞬間的速度
    bool flag = false;  // 第一個瞬間不積分的flag

    while( x_tor > x_tol_margin && y_tor > y_tol_margin && condition.isTouch() == false){
        // 計算誤差，publish 新速度
        x_tor = x_cor - x_now, y_tor = y_cor - y_now;

        vel_pub.x = x_tor*p_coe;
        vel_pub.y = y_tor*p_coe;
        vel_publisher.publish(vel_pub);

        // 接收 encoder
        ros::spinOnce();
        time_now = ros::Time::now().toSec();

        // 積分 (公分/秒) 
        if(flag){
            x_now += (time_now - time_before) * (vel_sub.x + x_vel_before)/2;
            y_now += (time_now - time_before) * (vel_sub.y + y_vel_before)/2; 
        }   flag = true;
        
        x_vel_before = vel_sub.x;
        y_vel_before = vel_sub.y;
        time_before = time_now;

        // 只要微動開關已經碰到牆壁，就立刻跳出迴圈
        if (condition.isTouch() == true) break;
    }

    // 到達目的地後靜止
        vel_pub.x = 0;
        vel_pub.y = 0;
        vel_publisher.publish(vel_pub);
}


/*** MoveTo -- Overloading TYPE 3 ***/
// moveTo(X,Y,Z)，XYZ 座標是「相對」於目前車身座標。
void moveTo(double x_cor, double y_cor, double z_cor){
    double x_now=0, y_now=0, z_now;  // 當前車身座標
    double x_tor = x_cor, y_tor = y_cor, z_tor=z_cor;  // 與目標差距
    double time_now, time_before;  
    double x_vel_before, y_vel_before, z_vel_before;  // 上個瞬間的速度
    bool flag = false;  // 第一個瞬間不積分的flag

    while( x_tor > x_tol_margin && y_tor > y_tol_margin && z_tor > z_tol_margin){
        // 計算誤差，publish 新速度
        x_tor = x_cor - x_now, y_tor = y_cor - y_now, z_tor = z_cor - z_now;

        vel_pub.x = x_tor*p_coe;
        vel_pub.y = y_tor*p_coe;
        vel_pub.z = z_tor*p_coe;
        vel_publisher.publish(vel_pub);

        // 接收 encoder
        ros::spinOnce();
        time_now = ros::Time::now().toSec();

        // 積分 (公分/秒) 
        if(flag){
            x_now += (time_now - time_before) * (vel_sub.x + x_vel_before)/2;
            y_now += (time_now - time_before) * (vel_sub.y + y_vel_before)/2; 
            z_now += (time_now - time_before) * (vel_sub.z + z_vel_before)/2; 
        }   flag = true;
        
        x_vel_before = vel_sub.x;
        y_vel_before = vel_sub.y;
        z_vel_before = vel_sub.z;
        time_before = time_now;

    }
    
    // 到達目的地後靜止
        vel_pub.x = 0;
        vel_pub.y = 0;
        vel_publisher.publish(vel_pub);
}


/*** MoveTo -- Overloading TYPE 4 ***/
// moveTo(X,Y,Z, MICRO)，XYZ 座標是「相對」於目前車身座標。
// MICRO 增加微動開關的判斷條件。
void moveTo(double x_cor, double y_cor, double z_cor, MICRO condition){
    double x_now=0, y_now=0, z_now;  // 當前車身座標
    double x_tor = x_cor, y_tor = y_cor, z_tor=z_cor;  // 與目標差距
    double time_now, time_before;  
    double x_vel_before, y_vel_before, z_vel_before;  // 上個瞬間的速度
    bool flag = false;  // 第一個瞬間不積分的flag

    while( x_tor > x_tol_margin && y_tor > y_tol_margin && z_tor > z_tol_margin && condition.isTouch() == false){
        // 計算誤差，publish 新速度
        x_tor = x_cor - x_now, y_tor = y_cor - y_now, z_tor = z_cor - z_now;

        vel_pub.x = x_tor*p_coe;
        vel_pub.y = y_tor*p_coe;
        vel_pub.z = z_tor*p_coe;
        vel_publisher.publish(vel_pub);

        // 接收 encoder
        ros::spinOnce();
        time_now = ros::Time::now().toSec();

        // 進行積分 (公分/秒) 
        if(flag){
            x_now += (time_now - time_before) * (vel_sub.x + x_vel_before)/2;
            y_now += (time_now - time_before) * (vel_sub.y + y_vel_before)/2; 
            z_now += (time_now - time_before) * (vel_sub.z + z_vel_before)/2; 
        }   flag = true;
        

        x_vel_before = vel_sub.x;
        y_vel_before = vel_sub.y;
        z_vel_before = vel_sub.z;
        time_before = time_now;
    
        // 只要微動開關已經碰到牆壁，就立刻跳出迴圈
        if(condition.isTouch() == true) break;
    }
    
    // 到達目的地後靜止
        vel_pub.x = 0;
        vel_pub.y = 0;
        vel_publisher.publish(vel_pub);
}
