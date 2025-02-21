#ifndef _BASIC_DEV_HPP_
#define _BASIC_DEV_HPP_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "airsim_ros/VelCmd.h"
#include "airsim_ros/PoseCmd.h"
#include "airsim_ros/Takeoff.h"
#include "airsim_ros/Reset.h"
#include "airsim_ros/Land.h"
#include "airsim_ros/GPSYaw.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/PointCloud2.h"
#include  "sensor_msgs/Imu.h"
#include <time.h>
#include <stdlib.h>
#include "Eigen/Dense"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <ros/callback_queue.h>
#include <boost/thread/thread.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include "airsim_ros/RotorPWM.h"
#include "DroneController.hpp"
#include "airsim_ros/dronecontrol.h"
#endif

class BasicDev
{
private:
    cv_bridge::CvImageConstPtr cv_bottom_ptr, cv_front_left_ptr, cv_front_right_ptr;
    cv::Mat front_left_img, front_right_img, bottom_img;

    std::unique_ptr<image_transport::ImageTransport> it;
    ros::CallbackQueue go_queue;
    ros::CallbackQueue front_img_queue;

    // 调用服务前需要定义特定的调用参数
    airsim_ros::Takeoff takeoff;
    airsim_ros::Land land;
    airsim_ros::Reset reset;

    // 使用publisher发布速度指令需要定义 Velcmd , 并赋予相应的值后，将他publish（）出去
    airsim_ros::VelCmd velcmd;
    airsim_ros::RotorPWM pwm_cmd;

    //无人机信息通过如下命令订阅，当收到消息时自动回调对应的函数
    ros::Subscriber odom_suber;//状态真值
    ros::Subscriber gps_suber;//gps数据
    ros::Subscriber imu_suber;//imu数据
    ros::Subscriber lidar_suber;//lidar数据

    //获取自己发出的话题
    ros::Subscriber odom_flu_nav_suber;
    float position_flu[3] = {0};

    float currentRoll;//当前滚转角
    float currentPitch;//当前俯仰角
    float currentYaw;//当前偏航角

    image_transport::Subscriber front_left_view_suber;
    image_transport::Subscriber front_right_view_suber;

    ros::Subscriber Keyboard_Control_suber;

    //通过这两个服务可以调用模拟器中的无人机起飞和降落命令
    ros::ServiceClient takeoff_client;
    ros::ServiceClient land_client;
    ros::ServiceClient reset_client;


    //PID控制器
    DroneContorller droneContorller;

    //通过publisher实现对无人机的控制
    ros::Publisher vel_publisher;
    ros::Publisher pwm_publisher;

    ros::Publisher control_cmd_publisher;

    //创建控制定时器
    ros::Timer control_timer;

    //临时任务列表
    bool task_list[10] = {false};
    float task_position[10][4] = {0};
    float task_yaw[10] = {0};
    int task_num = 0;

    bool is_takeoff = false;
    bool is_takeoff2 = false;

    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void gps_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);
    void lidar_cb(const sensor_msgs::PointCloud2::ConstPtr& msg);

    void front_left_view_cb(const sensor_msgs::ImageConstPtr& msg);
    void front_right_view_cb(const sensor_msgs::ImageConstPtr& msg);

    void control_cb(const ros::TimerEvent&);

    void keyboard_cb(const airsim_ros::VelCmd::ConstPtr& msg);

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);


public:
    BasicDev(ros::NodeHandle *nh);
    ~BasicDev();

};





