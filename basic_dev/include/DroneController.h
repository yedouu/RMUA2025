// DroneController.h

#ifndef DRONE_CONTROLLER_H
#define DRONE_CONTROLLER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>  // 引入 Odometry 消息类型
#include <airsim_ros/RotorPWM.h>  // 引入 RotorPWM 消息类型

// PID 控制器结构体
struct PID {
    float kp, ki, kd;   // PID 参数
    float prevError;    // 前一次误差
    float integ;        // 积分项
    float out;          // 控制输出

    // 默认构造函数
    PID() : kp(0), ki(0), kd(0), prevError(0), integ(0), out(0) {}

    // 带参数的构造函数
    PID(float Kp, float Ki, float Kd) : kp(Kp), ki(Ki), kd(Kd), prevError(0), integ(0), out(0) {}

    void update(float error, float dt);
};

// DroneController 类
class DroneController {
public:
    DroneController();

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

private:
    ros::NodeHandle nh;
    ros::Subscriber odom_sub;
    ros::Publisher pwm_pub;

    PID heightPID;  // 高度 PID 控制器

    float targetHeight;
    bool pidStarted;
    ros::Time startTime;
    float pwmValue;

    // 将PID输出映射到PWM信号范围 [0, 1]
    float mapToNormalizedPWM(float pidOutput, float minValue, float maxValue);

    // 动态调整 Kp：误差大时减小 Kp，误差小的时候增大 Kp
    void dynamicKpAdjustment(float error);
};

#endif  // DRONE_CONTROLLER_H
