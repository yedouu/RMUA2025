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

    void update(float error, float dt) {
        integ += error * dt;
        float deriv = (error - prevError) / dt;
        out = kp * error + ki * integ + kd * deriv;
        prevError = error;
    }
};

class DroneController {
public:
    DroneController() : pidStarted(false), startTime(ros::Time::now()) {
        // 初始化 PID 控制器
        // heightPID = PID(10.0, 2.5, 10.0);  // 外环PID (高度控制)
        // velocityPID = PID(0.5, 0.1, 0.2);  // 内环PID (速度控制)

        heightPID = PID(10.0, 1.0, 8.0);  // 外环PID (高度控制)
        velocityPID = PID(2.5, 0, 0.25);  // 内环PID (速度控制)
        


        // ROS 订阅器和发布器
        odom_sub = nh.subscribe("/odom_flu_nav", 10, &DroneController::odomCallback, this);  // 订阅 Odometry 数据
        pwm_pub = nh.advertise<airsim_ros::RotorPWM>("/airsim_node/drone_1/rotor_pwm_cmd", 10);  // 发布 PWM 信号

        // 目标高度
        targetHeight = 1.2;  // 假设目标高度为1米
        targetVelocity = 0.0;  // 目标垂直速度

        // 初始化 PWM 设置
        pwmValue = 0.18;
    }

    // 订阅 Odometry 数据（高度和速度）
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // 从 Odometry 数据中提取当前高度（pose.pose.position.z）
        float currentHeight = msg->pose.pose.position.z;
        float dt = (ros::Time::now() - prev_time).toSec();  // 计算时间间隔

        // 调试信息：显示时间间隔
        ROS_INFO("Time Interval (dt): %.4f", dt);

        // 如果尚未启动 PID 控制，则等待设定的延时
        if (!pidStarted && (ros::Time::now() - startTime).toSec() > 1.0) {  // 延时 1 秒
            pidStarted = true;  // 启动 PID 控制
            ROS_INFO("PID controller started.");
        }

        // 计算垂直速度（如果是第一次计算，忽略速度控制）
        float currentVelocity = 0.0;
        if (dt > 0.0) {
            currentVelocity = (currentHeight - prevHeight) / dt;  // 计算垂直速度
        }

        // 外环 PID：控制目标高度
        if (pidStarted) {
            // 计算高度误差
            float heightError = targetHeight - currentHeight;

            // 调试信息：输出高度误差和目标高度
            // ROS_INFO("Height Error: %.2f, Target Height: %.2f, Current Height: %.2f", heightError, targetHeight, currentHeight);

            heightPID.update(heightError, dt);

            // 外环PID的输出作为目标垂直速度
            targetVelocity = heightPID.out;

            // targetVelocity = 0.60;

            // 调试信息：输出目标速度
            ROS_INFO("Target Velocity (from PID): %.2f", targetVelocity);
        }


        // 内环 PID：控制速度误差，调整PWM信号
        float velocityError = targetVelocity - currentVelocity;
        velocityPID.update(velocityError, dt);

        // 调试信息：显示速度误差和当前速度
        ROS_INFO("Velocity Error: %.2f, Target Velocity: %.2f, Current Velocity: %.2f", velocityError, targetVelocity, currentVelocity);

        // 将内环PID输出映射到PWM范围
        pwmValue = mapToNormalizedPWM(velocityPID.out, -1.0, 1.0);  // 假设内环PID输出范围为-5到5

        // 调试信息：输出PID控制器的原始输出
        ROS_INFO("PID Output (Velocity): %.2f", velocityPID.out);

        // 限制PWM值范围
        if (pwmValue > 0.23) pwmValue = 0.23;
        if (pwmValue < 0.05) pwmValue = 0.05;

        // 调试信息：显示PWM值
        ROS_INFO("PWM Value (after limits): %.2f", pwmValue);

        // 计算四个电机的PWM值
        airsim_ros::RotorPWM pwm_msg;
        pwm_msg.rotorPWM0 = pwmValue;  // 右前电机
        pwm_msg.rotorPWM1 = pwmValue;  // 左后电机
        pwm_msg.rotorPWM2 = pwmValue;  // 左前电机
        pwm_msg.rotorPWM3 = pwmValue;  // 右后电机

        // 发布 PWM 信号
        pwm_pub.publish(pwm_msg);

        // 更新前一次高度和时间
        prevHeight = currentHeight;
        prev_time = ros::Time::now();  // 更新时间
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber odom_sub;
    ros::Publisher pwm_pub;

    // PID 控制器
    PID heightPID;  // 外环 PID 控制器（高度控制）
    PID velocityPID;  // 内环 PID 控制器（速度控制）

    // 目标高度和目标垂直速度
    float targetHeight;
    float targetVelocity;

    // 控制参数
    bool pidStarted;  // PID 是否已经启动
    ros::Time startTime;  // 启动时间
    ros::Time prev_time;  // 上一次的时间
    float pwmValue;  // 当前PWM值

    // 记录上次的高度
    float prevHeight;  // 上次的高度

    // 将PID输出映射到PWM信号范围 [0, 1]
    float mapToNormalizedPWM(float pidOutput, float minValue, float maxValue) {
        // 将PID输出范围从 [minValue, maxValue] 映射到 [0, 1]
        float normalizedOutput = (pidOutput - minValue) / (maxValue - minValue);

        // 确保归一化后的值在 [0, 1] 范围内
        if (normalizedOutput < 0) normalizedOutput = 0;
        if (normalizedOutput > 1) normalizedOutput = 1;

        return normalizedOutput;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "drone_height_pid_controller");
    DroneController controller;

    ros::spin();
    return 0;
}
