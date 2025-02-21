/*****************************************************************************************
 * 自定义控制器跟踪egoplanner轨迹
 * 本代码采用的mavros的速度控制进行跟踪
 * 编译成功后直接运行就行，遥控器先position模式起飞，然后rviz打点，再切offborad模式即可
 ******************************************************************************************/
#include <ros/ros.h> // 引入ROS头文件
#include <geometry_msgs/PoseStamped.h> // 引入带时间戳的姿态消息头文件
#include <geometry_msgs/Twist.h> // 引入速度消息头文件
#include <sensor_msgs/Joy.h> // 引入控制手柄消息头文件
#include "quadrotor_msgs/PositionCommand.h" // 引入四旋翼位置命令消息头文件
#include <nav_msgs/Odometry.h> // 引入里程计消息头文件
#include <tf/transform_datatypes.h> // 引入TF变换数据类型头文件
#include <tf/transform_broadcaster.h> // 引入TF变换广播器头文件
#include <tf2_ros/transform_listener.h> // 引入TF2变换监听器头文件
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // 引入TF2几何消息头文件

#include "DroneController.hpp"
#include "airsim_ros/dronecontrol.h"

class Ctrl
{
    public:
        Ctrl(); // 构造函数
        void position_cb(const nav_msgs::Odometry::ConstPtr &msg); // 位置回调函数
        void target_cb(const geometry_msgs::PoseStamped::ConstPtr& msg); // 目标回调函数
        void twist_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg); // 轨迹命令回调函数
        void control(const ros::TimerEvent&); // 控制函数
        ros::NodeHandle nh; // 节点句柄
        quadrotor_msgs::PositionCommand ego; // EGO规划器的信息
        tf::StampedTransform ts; // 用来发布无人机当前位置的坐标系坐标轴
        tf::TransformBroadcaster tfBroadcasterPointer; // 广播坐标轴
        nav_msgs::Odometry position_msg; // 位置消息
        geometry_msgs::PoseStamped target_pos; // 目标位置消息
        geometry_msgs::Twist vel_pub; // 速度消息
        geometry_msgs::Pose local_enu_pos;
        float position_x, position_y, position_z; // 无人机当前位置
        float now_x, now_y, now_yaw; // 当前参考点位置和航向
        float current_yaw; // 当前航向
        float targetpos_x, targetpos_y; // 目标位置
        float ego_pos_x = 0, ego_pos_y = 0, ego_pos_z = 1; // EGO规划器位置
        float ego_vel_x, ego_vel_y, ego_vel_z; // EGO规划器速度
        float ego_a_x, ego_a_y, ego_a_z; // EGO规划器加速度
        float ego_yaw, ego_yaw_rate; // EGO规划器航向和航向速率
        bool receive, get_now_pos; // 触发轨迹的条件判断
        ros::Subscriber state_sub, twist_sub, target_sub, position_sub,yaw_change_sub; // 订阅者
        ros::Publisher local_pos_pub, local_pos_pub2, pubMarker; // 发布者
        ros::Publisher local_vel_pub; // vel发布者
        ros::Timer timer; // 定时器

        ros::Publisher control_cmd_publisher;

        airsim_ros::dronecontrol control_cmd;
};

// 构造函数
Ctrl::Ctrl()
{
    timer = nh.createTimer(ros::Duration(0.03), &Ctrl::control, this); // 创建一个定时器，每0.02秒调用一次 control 函数
    position_sub = nh.subscribe("/odom_flu_nav", 10, &Ctrl::position_cb, this); // 订阅无人机位置消息
    target_sub = nh.subscribe("move_base_simple/goal", 10, &Ctrl::target_cb, this); // 订阅目标位置消息
    twist_sub = nh.subscribe("/position_cmd", 10, &Ctrl::twist_cb, this); // 订阅轨迹命令消息
    get_now_pos = false; // 初始化标志，表示是否获取了当前位置信息
    receive = false; // 初始化标志，表示是否接收到目标位置

    control_cmd_publisher = nh.advertise<airsim_ros::dronecontrol>("/drone_control", 1);
}

// 位置回调函数，更新当前位置信息
void Ctrl::position_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    position_msg = *msg; // 更新位置消息
    tf2::Quaternion quat; // 创建四元数对象
    tf2::convert(msg->pose.pose.orientation, quat); // 将姿态中的四元数转换到 tf2::Quaternion
    double roll, pitch, yaw; // 定义滚转角、俯仰角、航向角
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw); // 从四元数中提取出滚转角、俯仰角、航向角
    if (!get_now_pos) // 如果还没有获取当前位置信息
    {
        now_x = position_msg.pose.pose.position.x; // 更新当前参考点X坐标
        now_y = position_msg.pose.pose.position.y; // 更新当前参考点Y坐标
        tf2::Quaternion quat; // 创建四元数对象
        tf2::convert(msg->pose.pose.orientation, quat); // 将姿态中的四元数转换到 tf2::Quaternion
        now_yaw = yaw; // 更新当前航向角
        get_now_pos = true; // 标记已获取当前位置信息
    }
    position_x = position_msg.pose.pose.position.x; // 更新当前位置X坐标
    position_y = position_msg.pose.pose.position.y; // 更新当前位置Y坐标
    position_z = position_msg.pose.pose.position.z; // 更新当前位置Z坐标
    current_yaw = yaw; // 更新当前航向角
}

// 目标回调函数,接受rviz传来的目标点
void Ctrl::target_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    receive = true; // 标记已接收到目标位置
}

// 轨迹命令回调函数，接受EGO规划器的信息
void Ctrl::twist_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg)
{
    ego = *msg; // 更新EGO规划器的信息
    ego_pos_x = ego.position.x; // 更新EGO位置X坐标
    ego_pos_y = ego.position.y; // 更新EGO位置Y坐标
    ego_pos_z = ego.position.z; // 更新EGO位置Z坐标
    ego_vel_x = ego.velocity.x; // 更新EGO速度X分量
    ego_vel_y = ego.velocity.y; // 更新EGO速度Y分量
    ego_vel_z = ego.velocity.z; // 更新EGO速度Z分量
    ego_yaw = ego.yaw; // 更新EGO航向角
    
    ROS_WARN("规划速度更新！！！");
    // ROS_INFO("vel_x=%f,vel_y=%f,vel_z=%f,vel_yaw=%f",ego_vel_x,ego_vel_y,ego_vel_z,ego_yaw_rate);
    // ROS_INFO("pos_x=%f,pos_y=%f,pos_z=%f,pos_yaw=%f",ego_pos_x,ego_pos_y,ego_pos_z,ego_yaw);
    
}

// 控制函数
void Ctrl::control(const ros::TimerEvent&)
{
    if (!receive) // 如果没有接收到目标位置，就维持当前位置
    {
        
        control_cmd.yaw = now_yaw; // 设置航向角
        // control_cmd.yaw = 0; // 设置航向角

        control_cmd.px = now_x;
        control_cmd.py = now_y;
        control_cmd.pz = 2;


    }

    if (receive) // 如果接收到目标位置，就重新计算速度和yaw角并发布
    {
        
        control_cmd.yaw = -1*ego_yaw; // 设置航向角
        // control_cmd.yaw = 0; // 设置航向角
        
        control_cmd.px = ego_pos_x;
        control_cmd.py = ego_pos_y;
        control_cmd.pz = ego_pos_z;
        control_cmd.vx = ego_vel_x;
        control_cmd.vy = ego_vel_y;
        control_cmd.vz = ego_vel_z;
        control_cmd.mode = 0;   //位置控制
        // control_cmd.mode = 1;   //速度控制


    }
    control_cmd_publisher.publish(control_cmd); // 发布目标位置

}

// 主函数
int main(int argc, char **argv)
{
    ros::init(argc, argv, "RMUA_ego_ctrl"); // 初始化ROS节点
    setlocale(LC_ALL,""); // 设置本地化（确保中文显示正常）
    Ctrl ctrl; // 创建Ctrl对象
    std::cout<<"========================start_ctrl========================"<<std::endl; // 输出启动信息
    ros::spin(); // 进入ROS事件循环
    return 0; // 程序结束
}
