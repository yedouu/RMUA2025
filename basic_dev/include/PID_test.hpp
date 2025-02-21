#include <ros/ros.h>
#include <nav_msgs/Odometry.h>  // 引入 Odometry 消息类型
#include <airsim_ros/RotorPWM.h>  // 引入 RotorPWM 消息类型

class PID
{
private:
    float kp, ki, kd;   // PID 参数
    float prevError;    // 前一次误差
    float integ;        // 积分项
    float out;          // 控制输出
public:
    PID();
    PID(float Kp, float Ki, float Kd);
    ~PID();
    void setPID(float Kp, float Ki, float Kd);
    void update(float error, float dt);
    float getOut();
};

PID::PID():kp(0), ki(0), kd(0), prevError(0), integ(0), out(0)
{

}

PID::PID(float Kp, float Ki, float Kd)
    : kp(Kp), ki(Ki), kd(Kd), prevError(0), integ(0), out(0)
{
}

PID::~PID()
{
}

void PID::setPID(float Kp, float Ki, float Kd)
{
    kp = Kp;
    ki = Ki;
    kd = Kd;
}

void PID::update(float error, float dt)
{
    integ += error * dt;
    float deriv = (error - prevError) / dt;
    out = kp * error + ki * integ + kd * deriv;
    // ROS_INFO("kp*error:%f,error:%f,ki*integ:%f,kd*deriv:%f",kp*error,error,ki*integ,kd*deriv);
    prevError = error;

    //打印dt
    // ROS_INFO("dt:%f",dt);
}

float PID::getOut()
{
    return out;
}
