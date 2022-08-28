#include "ros/ros.h"
#include "std_msgs/String.h"

void chatCallback(const std_msgs::String::ConstPtr& msg) 
{
    // ROS_INFO("I heard: [%s]", msg->data.c_str());
    std::cout << msg->data << std::endl;
}

int main(int argc, char** argv) 
{

    ros::init(argc, argv, "listener");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("chat", 1, chatCallback);
    ros::spin();
    return 0;
}
