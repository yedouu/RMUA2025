#include "PID_test.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <sensor_msgs/Imu.h>

class DroneContorller{
private:
    ros::NodeHandle nh;


    //PID控制器初始化
    PID heightPID;  // 外环PID
    PID velocity_heightPID;  // 内环PID
    PID roll_vel_PID;  // roll角速度PID
    PID pitch_vel_PID;  // pitch角速度PID
    PID roll_PID; // roll角PID
    PID pitch_PID; // pitch角PID
    PID X_PID; // x位置PID
    PID Y_PID; // y位置PID
    PID X_vel_PID; // x速度PID
    PID Y_vel_PID; // y速度PID
    PID Yaw_vel_PID;    // yaw角速度PID
    PID Yaw_PID;    // yaw角PID

    // PID需要的反馈数据及其回调函数
    float currentHeight;    // 当前高度
    float currentVelocity_z;  // 当前z轴速度
    float current_XY[2];  // 当前x、y位置
    float currentVelocity_x;  // 当前x轴速度
    float currentVelocity_y;  // 当前y轴速度
    ros::Subscriber odom_sub;  // 里程计数据订阅器
    float currentRoll;//当前滚转角
    float currentPitch;//当前俯仰角
    float currentYaw;//当前偏航角
    float currentRoll_velocity;//当前滚转角速度
    float currentPitch_velocity;//当前俯仰角速度
    float currentYaw_velocity;//当前偏航角速度
    ros::Subscriber imu_sub;  // IMU数据订阅器


    //tf变换
    tf2_ros::Buffer tfBuffer;



    // PID所需要的目标值
    float targetHeight;     // 目标高度
    float targetVelocity_z;  // 目标z轴速度
    float targetXY[2];  // 目标x、y位置

    //其他数据
    ros::Time prev_time;  // 上一次的时间
    ros::Time prev_time2;  // 上一次的时间
    ros::Time prev_time3;  // 上一次的时间
    ros::Time prev_time4;  // 上一次的时间
    ros::Time prev_time5;  // 上一次的时间
    ros::Time prev_time6;  // 上一次的时间
    ros::Time prev_time7;  // 上一次的时间
    ros::Time prev_time8;  // 上一次的时间
    ros::Time startTime;  // 启动时间
    float prevHeight;  // 上次的高度
    float prevXY[2];  // 上次的x、y位置
    float pwmvalue_out_z;  // z轴电机输出


    //PID开始标志
    bool is_imu_receive = false;
    bool is_odom_receive = false;

public:
    DroneContorller(){
        // 订阅里程计数据
        odom_sub = nh.subscribe("/odom_flu_nav", 10, &DroneContorller::odomCallback, this);

        //订阅IMU数据
        imu_sub = nh.subscribe("/airsim_node/drone_1/imu/imu", 10, &DroneContorller::imuCallback, this);

        //初始化PID
        // heightPID.setPID(10.0, 0.001, 8.0);  
        // velocity_heightPID.setPID(6.0, 0.001, 0.2);

        heightPID.setPID(8.0, 0.15, 0.1);  
        velocity_heightPID.setPID(2.5, 0, 0.25);
        roll_vel_PID.setPID(5, 0.01, 0.001);
        pitch_vel_PID.setPID(5, 0.001, 0.001);
        roll_PID.setPID(0.5, 0, 0.001);
        pitch_PID.setPID(0.5, 0, 0.001);
        X_PID.setPID(0.2, 0, 0.05);
        Y_PID.setPID(0.2, 0, 0.05);
        X_vel_PID.setPID(0.13, 0, 0.005);
        Y_vel_PID.setPID(0.13, 0, 0.005);
        Yaw_vel_PID.setPID(7.0, 0, 0.001);
        Yaw_PID.setPID(0.5, 0, 0.001);

        
        //初始化时间
        startTime = prev_time = prev_time2 = prev_time3 = prev_time4 = ros::Time::now();


    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
        if(!is_odom_receive){
            is_odom_receive = true;
        }
        
        //读取XY位置
        current_XY[0] = msg->pose.pose.position.x;
        current_XY[1] = msg->pose.pose.position.y;

        // 读取高度
        currentHeight = msg->pose.pose.position.z;
            
    }
    
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
        if(!is_imu_receive){
            is_imu_receive = true;
        }
        
         // 读取欧拉角
        
        //计算欧拉角
        tf2::Quaternion quat; // 创建四元数对象
        tf2::convert(msg->orientation, quat); 
        double roll, pitch, yaw; // 定义滚转角、俯仰角、航向角
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw); // 从四元数中提取出滚转角、俯仰角、航向角
        // ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);
        currentRoll = roll;
        currentPitch = pitch;
        currentYaw = yaw;

        //打印欧拉角
        // ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f", currentRoll, currentPitch, currentYaw);
        
        //根据角度范围筛选不正常的值
        // if(currentRoll > 3.14) currentRoll = 0;
        // if(currentRoll < -3.14) currentRoll = 0;
        // if(currentPitch > 3.14) currentPitch = 0;
        // if(currentPitch < -3.14) currentPitch = 0;
        // if(currentYaw > 3.14) currentYaw = 0;
        // if(currentYaw < -3.14) currentYaw = 0;

        //获得角速度
        currentRoll_velocity = msg->angular_velocity.x;
        currentPitch_velocity = msg->angular_velocity.y;
        currentYaw_velocity = msg->angular_velocity.z;
        //根据正负值筛除不正常的角速度（不正常就赋为0）
        // if(currentRoll_velocity > 0.5) currentRoll_velocity = 0;
        // if(currentRoll_velocity < -0.5) currentRoll_velocity = 0;
        // if(currentPitch_velocity > 0.5) currentPitch_velocity = 0;
        // if(currentPitch_velocity < -0.5) currentPitch_velocity = 0;
        // if(currentYaw_velocity > 0.5) currentYaw_velocity = 0;
        // if(currentYaw_velocity < -0.5) currentYaw_velocity = 0;
    
    }

    // 将PID输出映射到PWM信号范围 [0, 1]
    float mapToNormalizedPWM(float pidOutput, float minValue, float maxValue) {
        // 将PID输出范围从 [minValue, maxValue] 映射到 [0, 1]
        float normalizedOutput = 0.23*(pidOutput - minValue) / (maxValue - minValue);

        // 确保归一化后的值在 [0, 1] 范围内
        if (normalizedOutput < 0) normalizedOutput = 0;
        if (normalizedOutput > 1) normalizedOutput = 1;

        return normalizedOutput;
    }

    //高度PID控制
    float heightPID_out(float targetHeight){
        // 计算时间差
        float dt = (ros::Time::now() - prev_time).toSec(); 

        // 计算高度误差
        float heightError = targetHeight - currentHeight;

        //打印高度误差
        // ROS_INFO("heightError: %f", heightError);

        //计算高度PID外环
        heightPID.update(heightError, dt);

        //将得到的输出作为速度目标
        targetVelocity_z = heightPID.getOut();

        //打印输出
        // ROS_INFO("targetVelocity_z: %f", targetVelocity_z);

        // 计算垂直速度（如果是第一次计算，忽略速度控制）
        if (dt > 0.0) {
            currentVelocity_z = (currentHeight - prevHeight) / dt;  // 计算垂直速度
        }

        //计算速度误差
        float velocityError = targetVelocity_z - currentVelocity_z;

        //计算速度PID内环
        velocity_heightPID.update(velocityError, dt);

        //将速度PID输出映射到电机输出
        pwmvalue_out_z = mapToNormalizedPWM(velocity_heightPID.getOut(), -1.0, 1.0);

        //保存当前高度和时间
        prevHeight = currentHeight;
        prev_time = ros::Time::now();

        // 限制PWM值范围
        if (pwmvalue_out_z > 0.23) pwmvalue_out_z = 0.23;
        if (pwmvalue_out_z < 0.05) pwmvalue_out_z = 0.05;

        return pwmvalue_out_z;
    }

    float height_velocityPID_out(float targetHeight_velocity){
        
        // 计算时间差
        float dt = (ros::Time::now() - prev_time2).toSec(); 

        // 计算垂直速度（如果是第一次计算，忽略速度控制）
        if (dt > 0.0) {
            currentVelocity_z = (currentHeight - prevHeight) / dt;  // 计算垂直速度
        }
        
        //打印当前z轴速度
        ROS_INFO("currentVelocity_z: %f", currentVelocity_z);

        //计算速度误差
        float velocityError = targetHeight_velocity - currentVelocity_z;

        //计算速度PID内环
        velocity_heightPID.update(velocityError, dt);

        //打印PID输出
        // ROS_INFO("velocity_heightPID_out: %f", velocity_heightPID.getOut());

        //将速度PID输出映射到电机输出
        pwmvalue_out_z = mapToNormalizedPWM(velocity_heightPID.getOut(), -1.0, 1.0);

        //保存当前高度和时间
        prevHeight = currentHeight;
        prev_time2 = ros::Time::now();

        // // 限制PWM值范围
        if (pwmvalue_out_z > 0.25) pwmvalue_out_z = 0.25;
        if (pwmvalue_out_z < 0.05) pwmvalue_out_z = 0.05;

        return pwmvalue_out_z;
    }

    //位置PID控制
    void XY_PID_out(float targetX, float targetY, float &X_pid_out, float &Y_pid_out){
        //计算时间差
        float dt = (ros::Time::now() - prev_time5).toSec();

        //计算x位置误差
        // float xError = targetX - current_XY[0];
        float xError = (targetX - current_XY[0])*cos(-1*currentYaw) + (targetY - current_XY[1])*sin(-1*currentYaw);


        //计算y位置误差
        // float yError = targetY - current_XY[1];
        float yError = -1*(targetX - current_XY[0])*sin(-1*currentYaw) + (targetY - current_XY[1])*cos(-1*currentYaw);
        
        //打印位置误差
        // ROS_INFO("xError:%f,yError:%f",xError,yError);

        //打印当前XY位置
        // ROS_INFO("current_X:%f,current_Y:%f",current_XY[0],current_XY[1]);

        //计算x位置PID输出
        X_PID.update(xError, dt);
        X_pid_out = X_PID.getOut();

        //计算y位置PID输出
        Y_PID.update(yError, dt);
        Y_pid_out = Y_PID.getOut();

        //限制输出范围
        if(X_pid_out > 1.0) X_pid_out = 1.0;
        if(X_pid_out < -1.0) X_pid_out = -1.0;
        if(Y_pid_out > 1.0) Y_pid_out = 1.0;
        if(Y_pid_out < -1.0) Y_pid_out = -1.0;

        //打印PID输出
        // ROS_INFO("X_PID_out:%f,Y_PID_out:%f",X_pid_out,Y_pid_out);

        //保存当前时间
        prev_time5 = ros::Time::now();
    }

    //速度PID控制
    void XY_vel_PID_out(float targetX_vel, float targetY_vel, float &X_pid_out, float &Y_pid_out){
        //计算时间差
        float dt = (ros::Time::now() - prev_time6).toSec();

        //计算XY方向速度
        if(dt >0){
            currentVelocity_x = (current_XY[0] - prevXY[0]) / dt;
            currentVelocity_y = (current_XY[1] - prevXY[1]) / dt;
        }
        //计算x速度误差
        // float x_velocityError = targetX_vel - currentVelocity_x;
        float x_velocityError = (targetX_vel - currentVelocity_x)*cos(-1*currentYaw) + (targetY_vel - currentVelocity_y)*sin(-1*currentYaw);
        //计算y速度误差
        // float y_velocityError = targetY_vel - currentVelocity_y;
        float y_velocityError = -1*(targetX_vel - currentVelocity_x)*sin(-1*currentYaw) + (targetY_vel - currentVelocity_y)*cos(-1*currentYaw);

        //打印速度误差
        // ROS_INFO("x_velocityError:%f,y_velocityError:%f",x_velocityError,y_velocityError);

        //打印当前XY速度
        // ROS_INFO("currentVelocity_x:%f,currentVelocity_y:%f",currentVelocity_x,currentVelocity_y);
        //计算x速度PID输出
        X_vel_PID.update(x_velocityError, dt);
        X_pid_out = X_vel_PID.getOut();

        //计算y速度PID输出
        Y_vel_PID.update(y_velocityError, dt);
        Y_pid_out = Y_vel_PID.getOut();

        //打印PID输出
        // ROS_INFO("X_vel_PID_out:%f,Y_vel_PID_out:%f",X_pid_out,Y_pid_out);

        //控制输出范围
        if(X_pid_out > 0.5) X_pid_out = 0.5;
        if(X_pid_out < -0.5) X_pid_out = -0.5;
        if(Y_pid_out > 0.5) Y_pid_out = 0.5;
        if(Y_pid_out < -0.5) Y_pid_out = -0.5;

        //打印PID输出
        // ROS_INFO("X_vel_PID_out:%f,Y_vel_PID_out:%f",X_pid_out,Y_pid_out);
        
        //保存当前时间和位置
        prev_time6 = ros::Time::now();
        prevXY[0] = current_XY[0];
        prevXY[1] = current_XY[1];
        
    }
    
    void attitude_vel_PID_out(float targetRoll_vel, float targetPitch_vel, float &Roll_pid_out, float &Pitch_pid_out){
        //计算时间差
        float dt = (ros::Time::now() - prev_time3).toSec();

        //计算滚转角速度误差
        float roll_velocityError = targetRoll_vel - currentRoll_velocity;
        //打印roll角速度误差
        // ROS_INFO("roll_velocityError:%f",roll_velocityError);

        //计算俯仰角速度误差
        float pitch_velocityError = targetPitch_vel - currentPitch_velocity;
        //打印pitch角速度误差
        // ROS_INFO("pitch_velocityError:%f",pitch_velocityError);

        //打印当前角速度
        // ROS_INFO("Roll_vel:%f,Pitch_vel:%f",currentRoll_velocity,currentPitch_velocity);

        //计算滚转角速度PID输出
        roll_vel_PID.update(roll_velocityError, dt);
        Roll_pid_out = roll_vel_PID.getOut();
        //限制输出范围
        // if(Roll_pid_out > 0.01) Roll_pid_out = 0.01;
        // if(Roll_pid_out < -0.01) Roll_pid_out = -0.01;


        //计算俯仰角速度PID输出
        pitch_vel_PID.update(pitch_velocityError, dt);
        Pitch_pid_out = pitch_vel_PID.getOut();
        //限制输出范围
        // if(Pitch_pid_out > 0.01) Pitch_pid_out = 0.01;
        // if(Pitch_pid_out < -0.01) Pitch_pid_out = -0.01;

        //保存当前时间
        prev_time3 = ros::Time::now();

    }

    void attitude_PID_out(float targetRoll, float targetPitch, float &Roll_pid_out, float &Pitch_pid_out){
        //计算时间差
        float dt = (ros::Time::now() - prev_time4).toSec();

        //计算Roll角误差
        float rollError = targetRoll - currentRoll;
        //打印roll角误差
        // ROS_INFO("rollError:%f",rollError);

        //计算Pitch角误差
        float pitchError = targetPitch - currentPitch;
        //打印pitch角误差
        // ROS_INFO("pitchError:%f",pitchError);

        //打印当前角度
        // ROS_INFO("Roll:%f,Pitch:%f",currentRoll,currentPitch);

        //计算Roll角PID输出
        roll_PID.update(rollError, dt);
        Roll_pid_out = roll_PID.getOut();
        //限制输出范围
        // if(Roll_pid_out > 0.01) Roll_pid_out = 0.01;
        // if(Roll_pid_out < -0.01) Roll_pid_out = -0.01;

        //计算Pitch角PID输出
        pitch_PID.update(pitchError, dt);
        Pitch_pid_out = pitch_PID.getOut();
        //限制输出范围
        // if(Pitch_pid_out > 0.01) Pitch_pid_out = 0.01;
        // if(Pitch_pid_out < -0.01) Pitch_pid_out = -0.01;

        //保存当前时间
        prev_time4 = ros::Time::now();

        
    }

    void Yaw_vel_PID_out(float targetYaw_vel, float &Yaw_vel_pid_out){
        //计算时间差
        float dt = (ros::Time::now() - prev_time7).toSec();

        //计算Yaw角速度误差
        float yaw_velocityError = targetYaw_vel - currentYaw_velocity;
        //打印yaw角速度误差
        // ROS_INFO("yaw_velocityError:%f",yaw_velocityError);

        //打印当前yaw角速度
        // ROS_INFO("Yaw_vel:%f",currentYaw_velocity);

        //计算Yaw角速度PID输出
        Yaw_vel_PID.update(yaw_velocityError, dt);
        Yaw_vel_pid_out = Yaw_vel_PID.getOut();
        //限制输出范围
        // if(Yaw_vel_pid_out > 0.01) Yaw_vel_pid_out = 0.01;
        // if(Yaw_vel_pid_out < -0.01) Yaw_vel_pid_out = -0.01;

        //保存当前时间
        prev_time7 = ros::Time::now();
    }

    void Yaw_PID_out(float targetYaw, float &Yaw_pid_out){
        //计算时间差
        float dt = (ros::Time::now() - prev_time8).toSec();

        //计算Yaw角误差和方向
        float yawError = fabs(targetYaw - currentYaw);
        if(yawError > 3.14) {
            yawError = 6.28 - yawError;
            if(targetYaw < currentYaw) yawError = -yawError;
        }
        else{
            if(targetYaw < currentYaw) yawError = -yawError;
        }
        

        //打印yaw角误差
        // ROS_INFO("yawError:%f",yawError);

        //打印当前yaw角
        // ROS_INFO("Yaw:%f",currentYaw);

        //计算Yaw角PID输出
        Yaw_PID.update(yawError, dt);
        Yaw_pid_out = Yaw_PID.getOut();
        //限制输出范围
        // if(Yaw_pid_out > 0.01) Yaw_pid_out = 0.01;
        // if(Yaw_pid_out < -0.01) Yaw_pid_out = -0.01;

        //保存当前时间
        prev_time8 = ros::Time::now();
    }

    void position_control(float targetX,float targetY,float targetZ,float targetYaw,float &pwm0,float &pwm1,float &pwm2,float &pwm3){
        if(!is_imu_receive || !is_odom_receive){
            ROS_INFO("wait for imu and odom data");
            return;
        }
        
        //打印目标位置
        ROS_INFO("targetX:%f,targetY:%f,targetZ:%f,targetYaw:%f",targetX,targetY,targetZ,targetYaw);
        
        //高度控制(计算油门)
        float thrust_out = heightPID_out(targetZ);

        //期望位置控制
        float X_pid_out, Y_pid_out;
        XY_PID_out(targetX, targetY, X_pid_out, Y_pid_out);


        //期望速度控制
        float X_vel_pid_out, Y_vel_pid_out;
        XY_vel_PID_out(X_pid_out, Y_pid_out, X_vel_pid_out, Y_vel_pid_out);
        // XY_vel_PID_out(0.2, 0, X_vel_pid_out, Y_vel_pid_out);

        //期望角度控制
        float roll_out, pitch_out;
        // attitude_PID_out(-0.02*Y_vel_pid_out, -0.02*X_vel_pid_out, roll_out, pitch_out);
        attitude_PID_out(-1*Y_vel_pid_out, -1*X_vel_pid_out, roll_out, pitch_out);
        // attitude_PID_out(0, 0.1, roll_out, pitch_out);

        //打印角度PID输出
        // ROS_INFO("roll_out:%f,pitch_out:%f",roll_out,pitch_out);

        //姿态控制(计算滚转角和俯仰角)
        float roll_vel_out, pitch_vel_out;
        attitude_vel_PID_out(roll_out, pitch_out, roll_vel_out, pitch_vel_out);

        //打印角速度PID输出
        // ROS_INFO("roll_vel_out:%f,pitch_vel_out:%f",roll_vel_out,pitch_vel_out);

        //期望偏航角控制
        float Yaw_pid_out;
        Yaw_PID_out(targetYaw, Yaw_pid_out);
        //打印偏航角PID输出
        // ROS_INFO("Yaw_pid_out:%f",Yaw_pid_out);

        //期望偏航角速度控制
        float Yaw_vel_pid_out;
        // Yaw_vel_PID_out(0.2, Yaw_vel_pid_out);
        Yaw_vel_PID_out(Yaw_pid_out, Yaw_vel_pid_out);

        //打印偏航角速度PID输出
        // ROS_INFO("Yaw_vel_pid_out:%f",Yaw_vel_pid_out);

        //控制分配
        pwm0 = thrust_out - roll_vel_out*0.002 + pitch_vel_out*0.002 + Yaw_vel_pid_out*0.002;
        pwm1 = thrust_out + roll_vel_out*0.002 - pitch_vel_out*0.002 + Yaw_vel_pid_out*0.002;
        pwm2 = thrust_out + roll_vel_out*0.002 + pitch_vel_out*0.002 - Yaw_vel_pid_out*0.002;
        pwm3 = thrust_out - roll_vel_out*0.002 - pitch_vel_out*0.002 - Yaw_vel_pid_out*0.002;

        // pwm0 = thrust_out ;
        // pwm1 = thrust_out ;
        // pwm2 = thrust_out ;
        // pwm3 = thrust_out ;

    }
};


