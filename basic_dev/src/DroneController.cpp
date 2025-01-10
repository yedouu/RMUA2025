// DroneController.cpp

#include "DroneController.h"

// PID 更新函数
void PID::update(float error, float dt) {
    integ += error * dt;
    float deriv = (error - prevError) / dt;
    out = kp * error + ki * integ + kd * deriv;
    prevError = error;
}

// DroneController 类构造函数
DroneController::DroneController() : pidStarted(false), startTime(ros::Time::now()) {
    // 初始化 PID 控制器
    heightPID = PID(10.0, 2.5, 10.0);  // 设置高度环 PID 参数

    // ROS 订阅器和发布器
    odom_sub = nh.subscribe("/odom_flu_nav", 10, &DroneController::odomCallback, this);  // 订阅Odometry话题
    pwm_pub = nh.advertise<airsim_ros::RotorPWM>("/airsim_node/drone_1/rotor_pwm_cmd", 10);  // 发布PWM信号

    // 目标高度
    targetHeight = 0.5;  // 假设目标高度为0.5米
}

// 订阅 Odometry 数据并计算 PWM
void DroneController::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    float currentHeight = msg->pose.pose.position.z;

    // 启动 PID 控制
    if (!pidStarted && (ros::Time::now() - startTime).toSec() > 1.0) {  
        pidStarted = true;  
    }

    if (pidStarted) {
        // 计算高度环 PID
        float heightError = targetHeight - currentHeight;
        float dt = 0.04;  // 假设控制周期为0.04秒
        heightPID.update(heightError, dt);

        // 将PID输出映射到PWM范围 [0, 1]
        pwmValue = mapToNormalizedPWM(heightPID.out, -10.0, 10.0);
    }

    if (pwmValue > 0.23) pwmValue = 0.23;  // 限制PWM范围在 [0, 0.23]
    
    // 计算四个电机的PWM值
    airsim_ros::RotorPWM pwm_msg;
    pwm_msg.rotorPWM0 = pwmValue;  
    pwm_msg.rotorPWM1 = pwmValue;  
    pwm_msg.rotorPWM2 = pwmValue;  
    pwm_msg.rotorPWM3 = pwmValue;  

    ROS_INFO("PWM: %.2f", pwmValue);

    pwm_pub.publish(pwm_msg);  
}

// 将 PID 输出映射到 PWM 信号范围 [0, 1]
float DroneController::mapToNormalizedPWM(float pidOutput, float minValue, float maxValue) {
    float normalizedOutput = (pidOutput - minValue) / (maxValue - minValue);
    if (normalizedOutput < 0) normalizedOutput = 0;
    if (normalizedOutput > 1) normalizedOutput = 1;
    return normalizedOutput;
}

// 动态调整 Kp（误差大时减小 Kp，误差小的时候增大 Kp）
void DroneController::dynamicKpAdjustment(float error) {
    if (fabs(error) > 0.2) {
        heightPID.kp = 5.0;  // 较小的 Kp
    } else if (fabs(error) > 0.05) {
        heightPID.kp = 8.0;  // 较大的 Kp
    } else {
        heightPID.kp = 12.0;  // 更大的 Kp
    }
}
