#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>  // 导入 Odometry 消息
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

ros::Publisher pub_transformed_pose;  // 发布器，用于发布转换后的坐标
ros::Publisher pub_odom;  // 发布器，用于发布 Odometry 消息

void callback(const geometry_msgs::PoseStamped::ConstPtr& msg, tf2_ros::Buffer& tfBuffer)
{
    try
    {
        // 创建 PointStamped 消息，用于存放输入和输出点
        geometry_msgs::PoseStamped point_in, point_out, point_out_flu;
        
        // 输入点的 header 和坐标
        point_in.header = msg->header;
        point_in.pose = msg->pose;
        
        // 设置原始坐标系（输入点所在的坐标系），比如 'base_link'
        point_in.header.frame_id = "world";  // 输入点所在的坐标系
        std::string target_frame = "origin";      // 目标坐标系

        
        geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform(
            target_frame,      // 目标坐标系
            point_in.header.frame_id,  // 源坐标系（输入点的坐标系）
            ros::Time(0)  // 查询最新的变换
        );

        // 执行坐标变换
        tf2::doTransform(point_in, point_out, transformStamped);

        //计算yaw角
        tf2::Quaternion q;
        tf2::convert(point_out.pose.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // 输出转换后的坐标
        ROS_INFO("Transformed Point: x=%.2f, y=%.2f, z=%.2f, yaw=%2f", 
                 point_out.pose.position.x, -1 * point_out.pose.position.y, -1 * point_out.pose.position.z, yaw);
        
        // 发布FLU坐标系下的里程计
        point_out_flu.header = point_out.header;
        point_out_flu.pose.position.x = point_out.pose.position.x;
        point_out_flu.pose.position.y = -1 * point_out.pose.position.y;
        point_out_flu.pose.position.z = -1 * point_out.pose.position.z;
        point_out_flu.pose.orientation = point_out.pose.orientation;

        // 发布转换后的 PoseStamped 坐标
        pub_transformed_pose.publish(point_out_flu);  

        // 创建并发布 Odometry 消息
        nav_msgs::Odometry odom_msg;
        odom_msg.header = point_out_flu.header;
        odom_msg.header.stamp = msg->header.stamp;
        odom_msg.header.frame_id = "origin";  // 设置参考坐标系

        // 里程计位置（从转换后的坐标系位置）
        odom_msg.pose.pose.position.x = point_out_flu.pose.position.x;
        odom_msg.pose.pose.position.y = point_out_flu.pose.position.y;
        odom_msg.pose.pose.position.z = point_out_flu.pose.position.z;
        odom_msg.pose.pose.orientation = point_out_flu.pose.orientation;

        // 可以设置速度部分，如果有必要的话，这里暂时设置为零
        odom_msg.twist.twist.linear.x = 0.0;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = 0.0;

        // 发布 Odometry 消息
        pub_odom.publish(odom_msg);

        // 发布 origin 到 body 的动态 tf 变换
        static tf2_ros::TransformBroadcaster broadcaster;
        geometry_msgs::TransformStamped tfs;
        tfs.header.frame_id = "origin";
        tfs.header.stamp = ros::Time::now();
        tfs.child_frame_id = "body";

        tfs.transform.translation.x = point_out_flu.pose.position.x;
        tfs.transform.translation.y = point_out_flu.pose.position.y;
        tfs.transform.translation.z = point_out_flu.pose.position.z;

        tfs.transform.rotation.x = point_out_flu.pose.orientation.x;
        tfs.transform.rotation.y = point_out_flu.pose.orientation.y;
        tfs.transform.rotation.z = point_out_flu.pose.orientation.z;
        tfs.transform.rotation.w = point_out_flu.pose.orientation.w;

        // 广播 TF 变换
        broadcaster.sendTransform(tfs);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could not transform point: %s", ex.what());
    }
}

int main(int argc, char** argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "coordinate_transform_node");
    ros::NodeHandle nh;

    // 创建 tf2 缓存和监听器
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // 创建发布器，用于发布转换后的坐标
    pub_transformed_pose = nh.advertise<geometry_msgs::PoseStamped>("/odom_flu", 10);
    
    // 创建 Odometry 消息发布器
    pub_odom = nh.advertise<nav_msgs::Odometry>("/odom_flu_nav", 10);

    // 订阅坐标点数据
    ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/airsim_node/drone_1/debug/pose_gt", 10, boost::bind(&callback, _1, boost::ref(tfBuffer))
    );

    // 让 ROS 节点持续运行，等待并处理回调
    ros::spin();

    return 0;
}
