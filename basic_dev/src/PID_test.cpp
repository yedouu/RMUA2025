#include "PID_test.hpp"

class DroneContorller{
private:
    ros::NodeHandle nh;


    //PID控制器初始化
    PID heightPID;  // 外环PID
    PID velocity_heightPID;  // 内环PID

    // PID需要的反馈数据及其回调函数
    float currentHeight;    // 当前高度
    float currentVelocity_z;  // 当前z轴速度
    ros::Subscriber odom_sub;  // 里程计数据订阅器


    // PID所需要的目标值
    float targetHeight;     // 目标高度
    float targetVelocity_z;  // 目标z轴速度

    //其他数据
    ros::Time prev_time;  // 上一次的时间
    ros::Time startTime;  // 启动时间
    float prevHeight;  // 上次的高度
    float pwmvalue_out_z;  // z轴电机输出

public:
    DroneContorller(){
        // 订阅里程计数据
        odom_sub = nh.subscribe("/odom_flu_nav", 10, &DroneContorller::odomCallback, this);

        //初始化PID
        heightPID.setPID(10.0, 1.0, 8.0);  
        velocity_heightPID.setPID(2.5, 0, 0.25);
        
        //初始化时间
        startTime = prev_time = ros::Time::now();


    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
        // 读取高度
        currentHeight = msg->pose.pose.position.z;

        // 计算时间差
        float dt = (ros::Time::now() - prev_time).toSec(); 

        // 调试信息：显示时间间隔
        ROS_INFO("Time Interval (dt): %.4f", dt);


    
    }
    
    // 将PID输出映射到PWM信号范围 [0, 1]
    float mapToNormalizedPWM(float pidOutput, float minValue, float maxValue) {
        // 将PID输出范围从 [minValue, maxValue] 映射到 [0, 1]
        float normalizedOutput = (pidOutput - minValue) / (maxValue - minValue);

        // 确保归一化后的值在 [0, 1] 范围内
        if (normalizedOutput < 0) normalizedOutput = 0;
        if (normalizedOutput > 1) normalizedOutput = 1;

        return normalizedOutput;
    }

    //高度PID控制
    float heightPID_out(float targetHeight){
        // 计算时间差
        float dt = (ros::Time::now() - prev_time).toSec(); 

        // 计算高度误差
        float heightError = targetHeight - currentHeight;

        //计算高度PID外环
        heightPID.update(heightError, dt);

        //将得到的输出作为速度目标
        targetVelocity_z = heightPID.getOut();

        //计算当前z轴速度
        float currentVelocity_z = (currentHeight - prevHeight) / dt;

        //计算速度误差
        float velocityError = targetVelocity_z - currentVelocity_z;

        //计算速度PID内环
        velocity_heightPID.update(velocityError, dt);

        //将速度PID输出映射到电机输出
        pwmvalue_out_z = mapToNormalizedPWM(velocity_heightPID.getOut(), -1.0, 1.0);

        return pwmvalue_out_z;
    }
};


