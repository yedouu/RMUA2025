#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.h>

ros::Publisher transformed_pc_pub;

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // 创建一个空的点云对象，使用智能指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr pts(new pcl::PointCloud<pcl::PointXYZ>);
    
    // 将 ROS PointCloud2 消息转换为 PCL PointCloud 格式
    pcl::fromROSMsg(*msg, *pts);
    
    // 创建一个新的点云用于存储转换后的数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pts(new pcl::PointCloud<pcl::PointXYZ>);

    // 坐标系转换：NED -> FLU
    for (size_t i = 0; i < pts->points.size(); ++i)
    {
        // 原始 NED 坐标系的点
        float x_ned = pts->points[i].x;
        float y_ned = pts->points[i].y;
        float z_ned = pts->points[i].z;

        // 坐标转换公式：NED -> FLU
        float x_flu = x_ned;     
        float y_flu = -y_ned;    
        float z_flu = -z_ned;    

        // 创建新的转换后的点
        pcl::PointXYZ new_point;
        new_point.x = x_flu;
        new_point.y = y_flu;
        new_point.z = z_flu;

        // 将转换后的点添加到新的点云中
        transformed_pts->points.push_back(new_point);
    }

    // 设置转换后的点云头信息
    transformed_pts->header = pts->header;

    // 将转换后的点云数据发布出去
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*transformed_pts, output);
    transformed_pc_pub.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_transformer");
    ros::NodeHandle nh;

    // 创建一个发布者，用于发布转换后的点云
    transformed_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/transformed_point_cloud", 1);

    // 订阅原始点云数据
    ros::Subscriber sub = nh.subscribe("airsim_node/drone_1/lidar", 1, pointCloudCallback);

    // 循环保持节点运行
    ros::spin();

    return 0;
}
