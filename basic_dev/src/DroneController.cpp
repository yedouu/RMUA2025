#include "airsim_ros/dronecontrol.h"  // 自定义的msg，用来传递控制信息
#include "ros/ros.h"
#include "DroneController.hpp"
#include "airsim_ros/RotorPWM.h"

class DroneControllerNode
{
public:
    DroneControllerNode()
    {
        // 初始化ROS节点和控制句柄
        nh_ = ros::NodeHandle();
        
        // 初始化目标位置和航向角
        target_position_[0] = target_position_[1] = target_position_[2] = 0;
        target_yaw_ = 0;
        
        // 初始化发布器
        pwm_publisher_ = nh_.advertise<airsim_ros::RotorPWM>("airsim_node/drone_1/rotor_pwm_cmd", 1);
        
        // 订阅控制消息
        control_subscriber_ = nh_.subscribe<airsim_ros::dronecontrol>("/drone_control", 1, &DroneControllerNode::controlMsgCallback, this);
        
        // 定时器控制
        control_timer_ = nh_.createTimer(ros::Duration(0.03), &DroneControllerNode::controlCallback, this);
    }

private:
    // ROS相关对象
    ros::NodeHandle nh_;
    ros::Publisher pwm_publisher_;
    ros::Subscriber control_subscriber_;
    ros::Timer control_timer_;
    
    // 控制信息
    float target_position_[3];
    float target_velocity_[3];
    float target_yaw_;
    int control_mode;
    airsim_ros::RotorPWM pwm_cmd_;
    
    // 控制器
    DroneContorller droneController_;

    //控制初始化标志
    bool is_control_init = false;
    
    // 控制信息回调
    void controlMsgCallback(const airsim_ros::dronecontrol::ConstPtr& msg)
    {
        target_position_[0] = msg->px;
        target_position_[1] = msg->py;
        target_position_[2] = msg->pz;

        target_velocity_[0] = msg->vx;
        target_velocity_[1] = msg->vy;
        target_velocity_[2] = msg->vz;

        target_yaw_ = msg->yaw;

        if(!is_control_init)
        {
            is_control_init = true;
        }
    }

    // 定时器控制回调
    void controlCallback(const ros::TimerEvent&)
    {
        if(!is_control_init)
        {
            return;
        }
        
        float pwm0, pwm1, pwm2, pwm3;
        // droneController_.position_control(target_position_[0], target_position_[1], target_position_[2], target_yaw_, pwm0, pwm1, pwm2, pwm3);

        if(control_mode == 0)
        {
            droneController_.position_control(target_position_[0], target_position_[1], target_position_[2], target_yaw_, pwm0, pwm1, pwm2, pwm3);
        }
        else if(control_mode == 1)
        {
            droneController_.velocity_control(target_velocity_[0], target_velocity_[1], target_velocity_[2], target_yaw_, pwm0, pwm1, pwm2, pwm3);
        }
        else
        {
            ROS_ERROR("Invalid control mode!");
        }

        pwm_cmd_.rotorPWM0 = pwm0;
        pwm_cmd_.rotorPWM1 = pwm1;
        pwm_cmd_.rotorPWM2 = pwm2;
        pwm_cmd_.rotorPWM3 = pwm3;
        
        pwm_publisher_.publish(pwm_cmd_);
    }
};  

int main(int argc, char** argv)
{
    ros::init(argc, argv, "DroneControlnode"); // 初始化ros 节点，命名为DroneControlnode
    
    // 创建DroneControllerNode对象
    DroneControllerNode drone_controller_node;
    
    // 循环等待回调
    ros::spin();
    
    return 0;
}
