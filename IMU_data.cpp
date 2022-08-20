#include <sensor_msgs/Imu.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){

    std::cout<< "x: " << msg->linear_acceleration.x << std::endl; 
    std::cout<< "y: " << msg->linear_acceleration.y << std::endl; 
    std::cout<< "z: " << msg->linear_acceleration.z << std::endl; 

}


int main(int argc, char ** argv){
    
    ros::init(argc, argv, "listener");
    ros::NodeHandle n; 
    ros::Subscriber imu_sub = n.subscribe("/kitti/oxts/imu", 10, imuCallback);
    
    ros::spin(); 

    return 0; 
}
