#include <iostream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"


int main(int argc, char** argv) {
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1);
    
    std::string path = "/home/jiwook/catkin_ws/src/src/img/Git.png";
  
    
    ros::Publisher img_pub = n.advertise<sensor_msgs::Image>("img",10);
    cv::Mat image = cv::imread(path, cv::IMREAD_COLOR);
    cv::imshow("show1",image);
    cv::waitKey(0);
    ros::Rate loop_rate(10);

   // sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    // int count = 0;
    
    sensor_msgs::Image img_msg;
    cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg(img_msg);
    while (ros::ok()) {
        std_msgs::String msg;
        msg.data = "jiwook";
        
        img_pub.publish(img_msg);

        chatter_pub.publish(msg);
        loop_rate.sleep();
        // ++count;
    }

    return 0;
}
