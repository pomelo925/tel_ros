    void run(){
        initial();

        phase1();
    }

// 初始化變數
    void initial(){
        ros::NodeHandle nh;
        x_vel_pub = nh.advertise<std_msgs::Float64>("X_vel",1);
        y_vel_pub = nh.advertise<std_msgs::Float64>("Y_vel",1);
        rot_pub = nh.advertise<std_msgs::Float64>("rot_vel",1);
    }


// moveTO(X,Y)，XY 座標是「相對」於目前車身座標。
    void moveTo(){
        
    }

// 第一階段：起點 ~ 影像辨識起點
    void phase1(){
        moveTo(-20,50);
    }

    void moveTO(){
        ROS_INFO("apple");
    }