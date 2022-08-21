#include <sensor_msgs/Imu.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <nav_msgs/Odometry.h>

double position_x;
double position_y;
double vel_x;
double vel_y;

ros::Time before_time;


nav_msgs::Odometry odom; 
ros::Publisher odom_pub;

double first_check_flag = false; 



void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{

    if(!first_check_flag){
        before_time=ros::Time::now();
        first_check_flag = true;
    }
    //std::cout<< "x: " << msg->linear_acceleration.x << std::endl; 
    //std::cout<< "y: " << msg->linear_acceleration.y << std::endl; 
    //std::cout<< "z: " << msg->linear_acceleration.z << std::endl; 
    double x_acc = msg->linear_acceleration.x;
    double y_acc = msg->linear_acceleration.y;
 
    ros::Duration del_t;
    del_t = ros::Time::now() - before_time;

    
    position_x = position_x + vel_x * del_t.toSec() + x_acc * 0.5 *del_t.toSec() * del_t.toSec();
    position_y = position_y + vel_y * del_t.toSec() + y_acc * 0.5 *del_t.toSec() * del_t.toSec();

    vel_x = vel_x +x_acc * del_t.toSec();
    vel_y = vel_y +y_acc * del_t.toSec();

    before_time = ros::Time::now();
    
    ROS_INFO("\npos_x : %.2f\npos_y: %.2f \nvel_x: %.2f\nvel_y:%.2f", position_x, position_y, vel_x, vel_y);
    
    odom.header.frame_id = "base_link";
    odom.pose.pose.position.x = position_x;
    odom.pose.pose.position.y = position_y;
    odom.pose.pose.position.z = 0;

    odom.pose.pose.orientation = msg -> orientation; 
    odom.twist.twist.linear.x = vel_x;
    odom.twist.twist.linear.y = vel_y;
    odom.twist.twist.linear.z = 0;

    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = 0;
    odom_pub.publish(odom);

}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle node; 

    odom_pub = node.advertise<nav_msgs::Odometry>("/odom/",10);

    ros::Subscriber imu_sub = node.subscribe("/kitti/oxts/imu", 10, imuCallback);
    ros::spin(); 
    return 0; 
}
