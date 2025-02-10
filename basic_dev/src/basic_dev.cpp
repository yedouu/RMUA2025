#ifndef _BASIC_DEV_CPP_
#define _BASIC_DEV_CPP_

#include "basic_dev.hpp"

int main(int argc, char** argv)
{

    ros::init(argc, argv, "basic_dev"); // 初始化ros 节点，命名为 basic
    ros::NodeHandle n; // 创建node控制句柄
    BasicDev go(&n);
    return 0;
}

BasicDev::BasicDev(ros::NodeHandle *nh)
{  
    //创建图像传输控制句柄
    it = std::make_unique<image_transport::ImageTransport>(*nh); 
    front_left_img = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0));
    front_right_img = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0));

    takeoff.request.waitOnLastTask = 1;
    land.request.waitOnLastTask = 1;

    // 使用publisher发布速度指令需要定义 Velcmd , 并赋予相应的值后，将他publish（）出去
    velcmd.twist.angular.z = 0;//z方向角速度(yaw, deg)
    velcmd.twist.linear.x = 0; //x方向线速度(m/s)
    velcmd.twist.linear.y = 0;//y方向线速度(m/s)
    velcmd.twist.linear.z = 0; //z方向线速度(m/s)

    pwm_cmd.rotorPWM0 = 0.1;
    pwm_cmd.rotorPWM1 = 0.1;
    pwm_cmd.rotorPWM2 = 0.1;
    pwm_cmd.rotorPWM3 = 0.1;

    control_cmd_publisher = nh->advertise<basic_dev::dronecontrol>("/drone_control", 1);

    //无人机信息通过如下命令订阅，当收到消息时自动回调对应的函数
    odom_suber = nh->subscribe<geometry_msgs::PoseStamped>("/airsim_node/drone_1/debug/pose_gt", 1, std::bind(&BasicDev::pose_cb, this, std::placeholders::_1));//状态真值，用于赛道一
    gps_suber = nh->subscribe<geometry_msgs::PoseStamped>("/airsim_node/drone_1/gps", 1, std::bind(&BasicDev::gps_cb, this, std::placeholders::_1));//状态真值，用于赛道一
    imu_suber = nh->subscribe<sensor_msgs::Imu>("airsim_node/drone_1/imu/imu", 1, std::bind(&BasicDev::imu_cb, this, std::placeholders::_1));//imu数据
    lidar_suber = nh->subscribe<sensor_msgs::PointCloud2>("airsim_node/drone_1/lidar", 1, std::bind(&BasicDev::lidar_cb, this, std::placeholders::_1));//imu数据
    front_left_view_suber = it->subscribe("airsim_node/drone_1/front_left/Scene", 1, std::bind(&BasicDev::front_left_view_cb, this,  std::placeholders::_1));
    front_right_view_suber = it->subscribe("airsim_node/drone_1/front_right/Scene", 1, std::bind(&BasicDev::front_right_view_cb, this,  std::placeholders::_1));
    //通过这两个服务可以调用模拟器中的无人机起飞和降落命令
    takeoff_client = nh->serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/takeoff");
    land_client = nh->serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/land");
    reset_client = nh->serviceClient<airsim_ros::Reset>("/airsim_node/reset");
    //通过publisher实现对无人机的控制
    vel_publisher = nh->advertise<airsim_ros::VelCmd>("airsim_node/drone_1/vel_cmd_body_frame", 1);
    pwm_publisher = nh->advertise<airsim_ros::RotorPWM>("airsim_node/drone_1/rotor_pwm_cmd", 1);
    
    control_timer = nh->createTimer(ros::Duration(0.03), &BasicDev::control_cb, this);

    Keyboard_Control_suber = nh->subscribe("/Vel_Control_debug", 1, &BasicDev::keyboard_cb, this);

    // is_takeoff = takeoff_client.call(takeoff); //起飞
    // land_client.call(land); //降落
    // reset_client.call(reset); //重置

    odom_flu_nav_suber = nh->subscribe("/odom_flu_nav", 10, &BasicDev::odomCallback, this);

    ros::spin();
}

BasicDev::~BasicDev()
{

}

void BasicDev::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    
    //读取坐标
    position_flu[0] = msg->pose.pose.position.x;
    position_flu[1] = msg->pose.pose.position.y;
    position_flu[2] = msg->pose.pose.position.z;

        
}

void BasicDev::control_cb(const ros::TimerEvent&)
{
    // if(is_takeoff){
    //     vel_publisher.publish(velcmd);
    //     // pwm_publisher.publish(pwm_cmd);
    // }

    // float pwnvalue_z = droneContorller.heightPID_out(1.0);
    // float pwnvalue_z = droneContorller.height_velocityPID_out(0.5);
    
    if(!is_takeoff2){
        //先确定任务
        task_position[0][0] = 0, task_position[0][1] = 0, task_position[0][2] = 1.0, task_position[0][3] = 0;
        task_position[1][0] = 0, task_position[1][1] = 0, task_position[1][2] = 2.0, task_position[1][3] = 0;
        task_position[2][0] = 1, task_position[2][1] = 0, task_position[2][2] = 2.0, task_position[2][3] = 0;
        task_position[3][0] = 2, task_position[3][1] = 0, task_position[3][2] = 2.0, task_position[3][3] = 0;
        is_takeoff2 = true;
    }

    //先计算与当前目标点的距离
    float distance = sqrt(pow(task_position[task_num][0] - position_flu[0], 2) + pow(task_position[task_num][1] - position_flu[1], 2) + pow(task_position[task_num][2] - position_flu[2], 2));
    if(distance < 0.1 and task_num < 3){    //还要加上航向角
        task_num++;
    }

    // float pwm0,pwm1,pwm2,pwm3;
    // // droneContorller.position_control(0, 0, 1.0, 1.57, pwm0, pwm1, pwm2, pwm3);
    // droneContorller.position_control(task_position[task_num][0],task_position[task_num][1] ,task_position[task_num][2] ,task_position[task_num][3] , pwm0, pwm1, pwm2, pwm3);

    // pwm_cmd.rotorPWM0 = pwm0;
    // pwm_cmd.rotorPWM1 = pwm1;
    // pwm_cmd.rotorPWM2 = pwm2;
    // pwm_cmd.rotorPWM3 = pwm3;
    // pwm_publisher.publish(pwm_cmd);

    basic_dev::dronecontrol control_cmd;
    control_cmd.px = task_position[task_num][0];
    control_cmd.py = task_position[task_num][1];
    control_cmd.pz = task_position[task_num][2];
    control_cmd.yaw = task_position[task_num][3];
    control_cmd_publisher.publish(control_cmd);

    
}

void BasicDev::keyboard_cb(const airsim_ros::VelCmd::ConstPtr& msg)
{
    ROS_INFO("Get keyboard data.");
    velcmd.twist.linear.x = msg->twist.linear.x;
    velcmd.twist.linear.y = msg->twist.linear.y;
    velcmd.twist.linear.z = msg->twist.linear.z;
    velcmd.twist.angular.z = msg->twist.angular.z;
}

void BasicDev::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Vector3d eulerAngle = q.matrix().eulerAngles(2,1,0);
    // ROS_INFO("Get pose data. time: %f, eulerangle: %f, %f, %f, posi: %f, %f, %f\n", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9,
    //     eulerAngle[0], eulerAngle[1], eulerAngle[2], msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void BasicDev::gps_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Vector3d eulerAngle = q.matrix().eulerAngles(2,1,0);
//     ROS_INFO("Get gps data. time: %f, eulerangle: %f, %f, %f, posi: %f, %f, %f\n", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9,
//         eulerAngle[0], eulerAngle[1], eulerAngle[2], msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void BasicDev::imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    //计算欧拉角
        tf2::Quaternion quat; // 创建四元数对象
        tf2::convert(msg->orientation, quat); 
        double roll, pitch, yaw; // 定义滚转角、俯仰角、航向角
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw); // 从四元数中提取出滚转角、俯仰角、航向角
        // ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);
        currentRoll = roll;
        currentPitch = pitch;
        currentYaw = yaw;
    
    // ROS_INFO("Get imu data. time: %f", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9);
}

void BasicDev::front_left_view_cb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_front_left_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
    if(!cv_front_left_ptr->image.empty())
    {
        cv::imshow("front_left", cv_front_left_ptr->image);
        cv::waitKey(10);
        // ROS_INFO("Get front left image.: %f", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9);
    }
    
}

void BasicDev::front_right_view_cb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_front_right_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
    if(!cv_front_right_ptr->image.empty())
    {
        // ROS_INFO("Get front right image.%f", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9);
    }
}

void BasicDev::lidar_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pts(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pts);
    // ROS_INFO("Get lidar data. time: %f, size: %ld", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9, pts->size());
}

#endif