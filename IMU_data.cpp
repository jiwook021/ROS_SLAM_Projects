#include <sensor_msgs/Imu.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <nav_msgs/Odometry.h>

double position_x;
double position_y;
double vel_x;
double vel_y;
double x_acc; 
ouble y_acc; 

ros::Time imu_before_time;
bool imu_first_check_flag = false; 
bool gps_first_check_flag = false; 

nav_msgs::Odometry imu_odom, gps_odom;
ros::Subscriber gps_sub;
ros::Publisher imu_odom_pub, gps_odom_pub;

double gps_ab_pos_x,gps_ab_pos_y,gps_ab_pos_z;

void gpsCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  if(!gps_first_check_flag)
  {
    imu_before_time = ros::Time::now();
    gps_ab_pos_x = msg->pose.pose.position.x;
    gps_ab_pos_y = msg->pose.pose.position.y;
    gps_ab_pos_z = msg->pose.pose.position.z;
    gps_first_check_flag = true;
  }
  else{
    std::cout << "gps_x: " << msg->pose.pose.position.x - gps_ab_pos_x <<std::endl; 
    std::cout << "gps_y: " << msg->pose.pose.position.y - gps_ab_pos_y <<std::endl; 
    std::cout << "gps_z: " << msg->pose.pose.position.z - gps_ab_pos_z <<std::endl; 

    gps_odom.header.frame_id = "base_link";

    gps_odom.pose.pose.position.x = msg->pose.pose.position.x - gps_ab_pos_x;
    gps_odom.pose.pose.position.y = msg->pose.pose.position.y - gps_ab_pos_y;
    gps_odom.pose.pose.position.z = msg->pose.pose.position.z - gps_ab_pos_z;
    gps_odom.pose.pose.orientation = msg ->pose.pose.orientation; 
    gps_odom.twist = msg -> twist;
   
    gps_odom_pub.publish(gps_odom);
  }
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{

    if(!imu_first_check_flag){
        imu_before_time = ros::Time::now();
        imu_first_check_flag = true;
    }
    //std::cout<< "x: " << msg->linear_acceleration.x << std::endl; 
    //std::cout<< "y: " << msg->linear_acceleration.y << std::endl; 
    //std::cout<< "z: " << msg->linear_acceleration.z << std::endl; 
    x_acc = msg->linear_acceleration.x;
    y_acc = msg->linear_acceleration.y;
 
    ros::Duration del_t;
    del_t = ros::Time::now() - imu_before_time;

    
    position_x = position_x + vel_x * del_t.toSec() + x_acc * 0.5 *del_t.toSec() * del_t.toSec();
    position_y = position_y + vel_y * del_t.toSec() + y_acc * 0.5 *del_t.toSec() * del_t.toSec();

    vel_x = vel_x +x_acc * del_t.toSec();
    vel_y = vel_y +y_acc * del_t.toSec();

<<<<<<< HEAD
    imu_before_time = ros::Time::now();
    ROS_INFO("\npos_x : %.2f\npos_y: %.2f \nvel_x: %.2f\nvel_y:%.2f", position_x, position_y, vel_x, vel_y);    
    imu_odom.header.frame_id = "base_link";
    imu_odom.pose.pose.position.x = position_x;
    imu_odom.pose.pose.position.y = position_y;
    imu_odom.pose.pose.position.z = 0;

    imu_odom.pose.pose.orientation = msg -> orientation; 
    imu_odom.twist.twist.linear.x = vel_x;
    imu_odom.twist.twist.linear.y = vel_y;
    imu_odom.twist.twist.linear.z = 0;

    imu_odom.twist.twist.angular.x = 0;
    imu_odom.twist.twist.angular.y = 0;
    imu_odom.twist.twist.angular.z = 0;
    imu_odom_pub.publish(imu_odom);
=======
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
>>>>>>> 388fc022698972b28cfa89f15b136112fbc99d21
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle node; 

    imu_odom_pub = node.advertise<nav_msgs::Odometry>("imu_odom/",10);
    gps_odom_pub = node.advertise<nav_msgs::Odometry>("gps_odom/",10);

<<<<<<< HEAD
    ros::Subscriber gps_sub = node.subscribe("GPS_nav", 10, gpsCallback);
    ros::Subscriber imu_sub = node.subscribe("/kitti/oxts/imu", 10, imuCallback);    
=======
    ros::Subscriber imu_sub = node.subscribe("/kitti/oxts/imu", 10, imuCallback);
    
>>>>>>> 388fc022698972b28cfa89f15b136112fbc99d21
    ros::spin(); 
    
    return 0; 
}

