#include <ros/ros.h>
#include <std_msgs/String.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include"geometry_msgs/PointStamped.h"
#include<string.h>
#include "airsim_ros/VelCmd.h"
 
class KeyboardInput
{
public:
    KeyboardInput()
    {
        tcgetattr(STDIN_FILENO, &initial_settings_);
        termios new_settings = initial_settings_;
        new_settings.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
        target_motion_pub = nh_.advertise<airsim_ros::VelCmd>(
            "/Vel_Control_debug", 1);
        
    }
    
    ~KeyboardInput()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &initial_settings_);
    }
    void run()
    {
        ros::Rate rate(10);
        while (ros::ok())
        {
            char c;
            if (read(STDIN_FILENO, &c, 1) == 1)
            {
                
                if(c=='z') break;

                //已获得键盘输入，开始获取坐标
                ROS_INFO("keyboard input:%c",c);

                

                //设置速度偏移量（NED坐标系）
                if(c == 'w') {
                    velcmd.twist.linear.x += 1.0;
                }
                if(c == 'x') {
                    velcmd.twist.linear.x -= 1.0;
                }
                if(c == 'a') {
                    velcmd.twist.linear.y -= 1.0;
                }
                if(c == 'd') {
                    velcmd.twist.linear.y += 1.0;
                }
                if(c == 'r') {
                    velcmd.twist.linear.z -= 1.0;
                }
                if(c == 'f') {
                    velcmd.twist.linear.z += 1.0;
                }

                //yaw角控制
                if(c == 'e') {
                    velcmd.twist.angular.z += 0.2;
                }
                if(c == 'q') {
                    velcmd.twist.angular.z -= 0.2;
                }
                if(c == 's') {  //悬停
                    velcmd.twist.linear.x = 0;
                    velcmd.twist.linear.y = 0;
                    velcmd.twist.linear.z = 0;
                    velcmd.twist.angular.z = 0;
                }
                

                
                
                target_motion_pub.publish(velcmd);

                ROS_INFO("velcmd: linear.x:%f, linear.y:%f, linear.z:%f, angular.z:%f", velcmd.twist.linear.x, velcmd.twist.linear.y, velcmd.twist.linear.z, velcmd.twist.angular.z);
            }
            ros::spinOnce();
            rate.sleep();
        }
    }
 
private:
    ros::NodeHandle nh_;
    termios initial_settings_;
    ros::Publisher local_pos_pub;
    ros::Subscriber odom_sub;
    double yaw_local_now;
    double rate_factor = 1.0;//速度因子
    ros::Publisher target_motion_pub;

    airsim_ros::VelCmd velcmd;
};
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_input_vel");
    KeyboardInput keyboard_input;
    keyboard_input.run();
    return 0;
}