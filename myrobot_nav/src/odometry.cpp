#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"

#include <boost/asio.hpp>
#include <boost/bind.hpp>
//#include<unistd.h>
#include <iostream>
#include <cstdio>
#include <cstring>
#include "string"

//#include "SerialPort.h"
extern "C" {
#include "serial.h"
#include "RobotControl.h"
}
using namespace std;

#define O_PI 3.1415926      //PI
#define WheelDiameter 0.071 //轮子直径
#define WheelWidth 0.625
//#define WheelWidth        0.286      //轮子间距

double x = 0, y = 0, theta = 0; //小车的位置和角度
double vx = 0, vy = 0, vth = 0; //小车的线速度和角速度

RobotInfo robotInfo;
double Dis = 0;
double LeftDis = 0;
double RightDis = 0;
//上一次左右轮里程
double lastLmileage = 0;
double lastRmileage = 0;

ros::Time currentTime;
ros::Time lastTime;
/************************
***************************
	    Twist转换为小车轮子速度信息
在此函数中：
把从cmd_vel主题中的Twist类型信息
转换为小车左右两轮的速度信息 单位:count/s
***************************************************/
void twistToWspd(const geometry_msgs::Twist &msg, float &lspd, float &rspd)
{
    geometry_msgs::Twist twist = msg;
    float vel_x = twist.linear.x;
     float vel_theta = twist.angular.z;

     vel_theta = vel_theta; //* (1 / Angular_Factor);

     float vel_left = 0, vel_right = 0;

     vel_left = vel_x - vel_theta * WheelWidth / 2.0;
     vel_right = vel_x + vel_theta * WheelWidth / 2.0;

     //   m/s
     lspd = vel_left;
     rspd = vel_right;

     /*
     //   rad/s
     float factor = O_PI*WheelDiameter;
     lspd /= factor;
     rspd /= factor;

    //   count/s
    lspd *= EncoderTicks;
    rspd *= EncoderTicks;

    //   count/5ms
    lspd /= 200;
    rspd /= 200;
    */
}
/***************************************************
	        cmd_vel回调函数
在此回调函数：
1.把Twist信息转换为小车两轮速度
2.通过串口把速度信息发给下位机
***************************************************/
void cmdVelCb(const geometry_msgs::Twist &msg)
{
    float lspd = 0, rspd = 0;
  
    twistToWspd(msg, lspd, rspd);
    MoveRobot(rspd, lspd);

    ROS_INFO("send speed msg L:%.1f R:%.1f", lspd, rspd);
}

//将里程计的速度转换为坐标系下的数据
int getOdomNew()

{
    //从下位机获取的机器人信息
    robotInfo = GetRobotInfo();
    //新增的左右轮距离
    double deltaLeftdis = (double)robotInfo.Lmileage - lastLmileage;
    double deltaRightdis = (double)robotInfo.Rmileage - lastRmileage;
    lastLmileage = robotInfo.Lmileage;
    lastRmileage = robotInfo.Rmileage;
    //新增的移动距离
    double deltaDis = 0.5f * (deltaLeftdis + deltaRightdis);
    //转角
    double deltaTheta = (double)(deltaRightdis - deltaLeftdis) / WheelWidth;

    //修正参数
    //deltaDis = deltaDis * Linear_Factor;
    //deltaLeftdis = deltaLeftdis*Linear_Factor;
    //deltaRightdis = deltaRightdis*Linear_Factor;
    //deltaTheta = deltaTheta * Angular_Factor;

    Dis += deltaDis;
    LeftDis += deltaLeftdis;
    RightDis += deltaRightdis;

    double deltaX = deltaDis * (double)cos(theta);
    double deltaY = deltaDis * (double)sin(theta);

    x += deltaX;
    y += deltaY;
    theta += deltaTheta;

    currentTime = ros::Time::now();
    double dt = (currentTime - lastTime).toSec();

    vx = deltaX / dt;
    vy = deltaY / dt;
    vth = deltaTheta / dt;

    if (theta > O_PI)
        theta -= 2 * O_PI;
    else if (theta < -O_PI)
        theta += 2 * O_PI;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "OdometryNode");
     RobotControlInit();
    ros::NodeHandle nodehandle;
    tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans;
    geometry_msgs::Quaternion odom_quat;
  
    nav_msgs::Odometry odom;
    ros::Publisher odom_pub = nodehandle.advertise<nav_msgs::Odometry>("odom", 100);
    ros::Subscriber sub = nodehandle.subscribe("cmd_vel", 1, &cmdVelCb);
    ros::Rate rate(20);

    //tf坐标
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    //里程计数据
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    while (ros::ok())
    {
        ros::spinOnce();

        //读取编码器信息并转换为里程计数据
        lastTime = ros::Time::now();
        getOdomNew();
        //发布TF消息
        ros::Time current_time = ros::Time::now();
        odom_quat = tf::createQuaternionMsgFromYaw(theta);

        odom_trans.header.stamp = current_time;
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        odom_broadcaster.sendTransform(odom_trans);

        //发布里程消息
        odom.header.stamp = current_time;
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;
        odom_pub.publish(odom);
        //  rate.sleep();
  }

  return 0;
}
