#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/PoseStamped.h"

class PoseTransformer {
public:
    PoseTransformer() : is_init_pose_get(false) {
        // 初始化ROS节点句柄
        ros::NodeHandle nh;

        // 订阅初始位姿和实时位姿
        
        init_pose_sub = nh.subscribe("/airsim_node/initial_pose", 10, &PoseTransformer::initPoseCallback, this);

        // 创建静态坐标转换广播器
        broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>();

        // 设置机体和雷达之间的tf坐标变换
        geometry_msgs::TransformStamped ts;
        ts.header.seq = 100;
        ts.header.stamp = ros::Time::now();
        ts.header.frame_id = "body";
        ts.child_frame_id = "lidar";
        ts.transform.translation.x = 0;
        ts.transform.translation.y = 0;
        ts.transform.translation.z = 0.05;

        


        tf2::Quaternion qtn;
        qtn.setRPY(0, 0, 0);
        ts.transform.rotation.x = qtn.getX();
        ts.transform.rotation.y = qtn.getY();
        ts.transform.rotation.z = qtn.getZ();
        ts.transform.rotation.w = qtn.getW();

        broadcaster->sendTransform(ts);
    }

    

    void initPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose) {
        if (!is_init_pose_get) {
            init_pose = *pose;
            ROS_INFO("Init pose get: %f %f %f", init_pose.pose.position.x, init_pose.pose.position.y, init_pose.pose.position.z);
            is_init_pose_get = true;
        }
        ROS_INFO("Init pose get: %f %f %f", init_pose.pose.position.x, init_pose.pose.position.y, init_pose.pose.position.z);
        //构建world到origin的tf变换
        geometry_msgs::TransformStamped ts;
        ts.header.seq = 100;
        ts.header.stamp = ros::Time::now();
        ts.header.frame_id = "world";
        ts.child_frame_id = "origin";

        ts.transform.translation.x = pose->pose.position.x;
        ts.transform.translation.y = pose->pose.position.y;
        ts.transform.translation.z = pose->pose.position.z;
        ts.transform.rotation.x = pose->pose.orientation.x;
        ts.transform.rotation.y = pose->pose.orientation.y;
        ts.transform.rotation.z = pose->pose.orientation.z;
        ts.transform.rotation.w = pose->pose.orientation.w;

        broadcaster->sendTransform(ts);
    }

private:
    bool is_init_pose_get;
    geometry_msgs::PoseStamped init_pose;

    ros::Subscriber init_pose_sub;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster;
    tf2_ros::TransformBroadcaster br;  // 用于发布动态坐标变换
};

int main(int argc, char *argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "tf_broadcast");

    PoseTransformer pose_transformer;

    ros::spin();

    return 0;
}
