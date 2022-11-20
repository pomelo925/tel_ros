#include "race/mecanum.h"

// declare NodeHandle and pub/sub
void MECANUM::init(){
    ros::NodeHandle nh_4mecanum;
    mecanum_publisher = nh_4mecanum.advertise<geometry_msgs::Point>("mecanum_toSTM", 1);
    mecanum_subscriber = nh_4mecanum.subscribe("mecanum_fromSTM", 1, MECANUM::callback);
}

// encdoer callback function and publish
void MECANUM::callback(const geometry_msgs::Point::ConstPtr &vel){
    mecanum_sub.x = vel->x;
    mecanum_sub.y = vel->y;
    mecanum_sub.z = vel->z;

    data_check = true;
}

void MECANUM::moveTo(double x_cor, double y_cor, double z_cor){
/* Clibration Coefficient */
    if(x_cor!=0) x_cor += calibration_x_intercept;
    if(y_cor!=0) y_cor += calibration_y_intercept;
    if(z_cor!=0) z_cor += calibration_z_intercept;

    x_cor /= calibration_x;
    y_cor /= calibration_y;
    z_cor /= calibration_z;

    z_cor *= 3.1415926 / 180;
    double acc_x = 0, acc_y = 0, acc_z = 0;
    double x_now = 0, y_now = 0, z_now = 0;
    double x_err = x_cor, y_err = y_cor, z_err = z_cor; // distance between now & goal
    double time_now, time_before;
    double x_vel_before, y_vel_before, z_vel_before; // velocity of previous instance
    bool flag = false;                               // flag for NOT integral on first instance

    while (fabs(x_err) > x_tol_margin || fabs(y_err) > y_tol_margin || fabs(z_err) > z_tol_margin){
        // calculate error and pub new speed
        x_err = x_cor - x_now;
        y_err = y_cor - y_now;
        z_err = z_cor - z_now;

        /* velocity profile */
        mecanum_pub.x = 0;
        mecanum_pub.y = 0;
        mecanum_pub.z = 0;

        /// accerlation ///
        if (fabs(x_err) > fabs(fod * x_cor) && fabs(x_err) > x_tol_margin){
            mecanum_pub.x = acc_x;
            acc_x += (x_err > 0) ? acc_xy : -acc_xy;
            if (acc_x >= max_xy) mecanum_pub.x = max_xy;
            if (acc_x <= -max_xy) mecanum_pub.x = -max_xy;
        }

        if (fabs(y_err) > fabs(fod * y_cor) && fabs(y_err) > y_tol_margin){
            mecanum_pub.y = acc_y;
            acc_y += (y_err > 0) ? acc_xy : -acc_xy;
            if (acc_y >= max_xy) mecanum_pub.y = max_xy;
            if (acc_y <= -max_xy) mecanum_pub.y = -max_xy;
        }

        if (fabs(z_err) > fabs(fod * z_cor) && fabs(z_err) > z_tol_margin){
            mecanum_pub.z = acc_zz;
            acc_z += (z_err > 0) ? acc_zz : -acc_zz;
            if (acc_z >= max_z) mecanum_pub.z = max_z;
            if (acc_z <= -max_z) mecanum_pub.z = -max_z;
        }

        /// deceleration ///
        if (fabs(x_err) <= fabs(fod * x_cor) && x_cor != 0){
            mecanum_pub.x = kp_xy * x_err;
            if (mecanum_pub.x > max_xy) mecanum_pub.x = max_xy;
        }
        if (fabs(y_err) <= fabs(fod * y_cor) && y_cor != 0){
            mecanum_pub.y = kp_xy * y_err;
            if (mecanum_pub.y > max_xy) mecanum_pub.y = max_xy;
        }
        if (fabs(z_err) <= fabs(fod * z_cor) && z_cor != 0){
            mecanum_pub.z = kp_z * z_err;
            if (mecanum_pub.z > max_z) mecanum_pub.z = max_z;
        }
        mecanum_publisher.publish(mecanum_pub);


        /* velocity profile */
        data_check = false;
        while (!data_check)
            ros::spinOnce();

        // integral (unit: cm/s)
        time_now = ros::Time::now().toSec();

        if (flag){
            x_now += (time_now - time_before) * (mecanum_sub.x + x_vel_before) / 2;
            y_now += ((time_now - time_before) * (mecanum_sub.y + y_vel_before) / 2);
            z_now += ((time_now - time_before) * (mecanum_sub.z + z_vel_before) / 2);
        }
        flag = true;

        std::cout << "X: " << x_now << "\t\tVx: " << mecanum_pub.x << std::endl;
        std::cout << "Y: " << y_now << "\t\tVy: " << mecanum_pub.y << std::endl;
        std::cout << "Z: " << z_now << "\t\tVz: " << mecanum_pub.z << std::endl;
        std::cout << "= = = = = = = = =" << std::endl;

        x_vel_before = mecanum_sub.x;
        y_vel_before = mecanum_sub.y;
        z_vel_before = mecanum_sub.z;
        time_before = time_now;
        ros::Duration(0.01).sleep();
    }

    // reaching goal and pub speed 0
    while (mecanum_sub.x != 0 || mecanum_sub.y != 0 || mecanum_sub.z != 0)
    {
        mecanum_pub.x = 0;
        mecanum_pub.y = 0;
        mecanum_pub.z = 0;
        mecanum_publisher.publish(mecanum_pub);

        data_check = false;
        while (!data_check)
            ros::spinOnce();

        std::cout << "X: " << x_now << "\t\tVx: " << mecanum_pub.x << std::endl;
        std::cout << "Y: " << y_now << "\t\tVy: " << mecanum_pub.y << std::endl;
        std::cout << "Z: " << z_now << "\t\tVz: " << mecanum_pub.z << std::endl;
        std::cout << "+ + + about to stop + + +" << std::endl;
    }
}

void MECANUM::moveTo(POINT point){
/* Clibration Coefficient */
    point.x_cor += calibration_x_intercept;
    point.y_cor += calibration_y_intercept;
    point.z_cor += calibration_z_intercept;
    
    point.x_cor /= calibration_x;
    point.y_cor /= calibration_y;
    point.z_cor /= calibration_z;  


    double acc_x = 0, acc_y = 0, acc_z = 0;
    double x_now = 0, y_now = 0, z_now = 0;
    double x_err = point.x_cor, y_err = point.y_cor, z_err = point.z_cor * 3.1415926 / 180; // distance between now & goal
    double time_now, time_before;
    double x_vel_before, y_vel_before, z_vel_before; // velocity of previous instance
    bool flag = false;                               // flag for NOT integral on first instance

    while (fabs(x_err) > x_tol_margin || fabs(y_err) > y_tol_margin || fabs(z_err) > z_tol_margin)
    {
        // calculate error and pub new speed
        x_err = point.x_cor - x_now, y_err = point.y_cor - y_now, z_err = point.z_cor - z_now;

        /* velocity profile */
        mecanum_pub.x = 0;
        mecanum_pub.y = 0;
        mecanum_pub.z = 0;

        /// accerlation ///
        if (fabs(x_err) > fabs(fod * point.x_cor) && fabs(x_err) > x_tol_margin){
            mecanum_pub.x = acc_x;
            acc_x += (x_err > 0) ? acc_xy : -acc_xy;
            if (acc_x >= max_xy) mecanum_pub.x = max_xy;
            if (acc_x <= -max_xy) mecanum_pub.x = -max_xy;
        }

        if (fabs(y_err) > fabs(fod * point.y_cor) && fabs(y_err) > y_tol_margin){
            mecanum_pub.y = acc_y;
            acc_y += (y_err > 0) ? acc_xy : -acc_xy;
            if (acc_y >= max_xy) mecanum_pub.y = max_xy;
            if (acc_y <= -max_xy) mecanum_pub.y = -max_xy;
        }

        if (fabs(z_err) > fabs(fod * point.z_cor) && fabs(z_err) > z_tol_margin){
            mecanum_pub.z = acc_zz;
            acc_z += (z_err > 0) ? acc_zz : -acc_zz;
            if (acc_z >= max_z) mecanum_pub.z = max_z;
            if (acc_z <= -max_z) mecanum_pub.z = -max_z;
        }

        /// deceleration ///
        if (fabs(x_err) <= fabs(fod * point.x_cor) && point.x_cor != 0){
            mecanum_pub.x = kp_xy * x_err;
            if (mecanum_pub.x > max_xy) mecanum_pub.x = max_xy;
        }
        if (fabs(y_err) <= fabs(fod * point.y_cor) && point.y_cor != 0){
            mecanum_pub.y = kp_xy * y_err;
            if (mecanum_pub.y > max_xy) mecanum_pub.y = max_xy;
        }
        if (fabs(z_err) <= fabs(fod * point.z_cor) && point.z_cor != 0){
            mecanum_pub.z = kp_z * z_err;
            if (mecanum_pub.z > max_z) mecanum_pub.z = max_z;
        }
        mecanum_publisher.publish(mecanum_pub);

        /* velocity profile */

        // MECANUM::TF(x_now, y_now, z_now, ros::Time::now());
        data_check = false;
        while (!data_check)
            ros::spinOnce();

        // integral (unit: cm/s)
        time_now = ros::Time::now().toSec();
        if (flag){
            x_now += (time_now - time_before) * (mecanum_sub.x + x_vel_before) / 2;
            y_now += ((time_now - time_before) * (mecanum_sub.y + y_vel_before) / 2);
            z_now += ((time_now - time_before) * (mecanum_sub.z + z_vel_before) / 2);
        }
        flag = true;

        std::cout << "X: " << x_now << "\t\tVx: " << mecanum_pub.x << std::endl;
        std::cout << "Y: " << y_now << "\t\tVy: " << mecanum_pub.y << std::endl;
        std::cout << "Z: " << z_now << "\t\tVz: " << mecanum_pub.z << std::endl;
        std::cout << "= = = = = = = = =" << std::endl;

        x_vel_before = mecanum_sub.x;
        y_vel_before = mecanum_sub.y;
        z_vel_before = mecanum_sub.z;
        time_before = time_now;
        ros::Duration(0.01).sleep();
    }

    // reaching goal and pub speed 0
    while (mecanum_sub.x != 0 || mecanum_sub.y != 0 || mecanum_sub.z != 0)
    {
        mecanum_pub.x = 0;
        mecanum_pub.y = 0;
        mecanum_pub.z = 0;
        mecanum_publisher.publish(mecanum_pub);

        data_check = false;
        while (!data_check)
            ros::spinOnce();

        std::cout << "X: " << x_now << "\t\tVx: " << mecanum_pub.x << std::endl;
        std::cout << "Y: " << y_now << "\t\tVy: " << mecanum_pub.y << std::endl;
        std::cout << "Z: " << z_now << "\t\tVz: " << mecanum_pub.z << std::endl;
        std::cout << "+ + + about to stop + + +" << std::endl;
    }
}