#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <gazebo_msgs/ModelStates.h>
#include <iostream>
#include <vector>
#include <sstream>
#include <cmath>
#include <thread>
#include <mutex>
#include <atomic>

// PID Controller class
class PIDController {
public:
    PIDController(double kp, double ki, double kd, double dt)
        : kp_(kp), ki_(ki), kd_(kd), dt_(dt), integral_(0), prev_error_(0) {}

    double compute(double setpoint, double measurement) {
        double error = setpoint - measurement;
        integral_ += error * dt_;
        double derivative = (error - prev_error_) / dt_;
        prev_error_ = error;
        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

    void setGains(double kp, double ki, double kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
        integral_ = 0; // Reset integral when gains change
    }

private:
    double kp_, ki_, kd_, dt_, integral_, prev_error_;
};

// Global variables
std::string drone_name;
geometry_msgs::Twist cmd_vel;
geometry_msgs::Point current_pos, target_pos;
std::atomic<bool> target_set(false); // Thread-safe flag for target state
std::atomic<bool> position_received(false); // Track if we’re getting position updates
std::atomic<bool> stop_pid_thread(false); // Flag to stop PID thread
std::atomic<bool> shutdown(false); // Flag to stop callback thread
const double threshold = 0.1; // Distance threshold to consider target reached
std::mutex data_mutex; // Protect shared data (cmd_vel, current_pos, target_pos)

// PID controllers for X, Y, Z axes (initial gains: kp=0.5, ki=0.01, kd=0.1, dt=0.1)
PIDController pid_x(0.5, 0.01, 0.1, 0.1);
PIDController pid_y(0.5, 0.01, 0.1, 0.1);
PIDController pid_z(0.5, 0.01, 0.1, 0.1);

// Publisher (will be initialized in main)
ros::Publisher vel_pub;

// Callback to get the drone's current position from Gazebo
void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex);
    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == drone_name) {
            current_pos = msg->pose[i].position;
            position_received = true;
            ROS_INFO_THROTTLE(1.0, "Received position: (%.2f, %.2f, %.2f)", 
                              current_pos.x, current_pos.y, current_pos.z);
            break;
        }
    }
}

// Callback processing thread
void callbackThread() {
    ros::Rate rate(10); // 10 Hz for callback processing
    ROS_INFO("Callback thread started");
    while (ros::ok() && !shutdown) {
        ros::spinOnce(); // Process callbacks
        rate.sleep();
    }
    ROS_INFO("Callback thread stopped");
}

// PID control thread function
void pidControlThread() {
    ros::Rate rate(10); // 10 Hz loop rate for PID updates
    ROS_INFO("PID thread started");
    while (ros::ok() && target_set && !stop_pid_thread) {
        std::lock_guard<std::mutex> lock(data_mutex);
        if (position_received) {
            cmd_vel.linear.x = pid_x.compute(target_pos.x, current_pos.x);
            cmd_vel.linear.y = pid_y.compute(target_pos.y, current_pos.y);
            cmd_vel.linear.z = pid_z.compute(target_pos.z, current_pos.z);
            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;
            cmd_vel.angular.z = 0.0;

            // Check if target is reached
            double dist = std::sqrt(std::pow(target_pos.x - current_pos.x, 2) +
                                    std::pow(target_pos.y - current_pos.y, 2) +
                                    std::pow(target_pos.z - current_pos.z, 2));
            ROS_INFO("Distance to target: %.2f, Velocity: (%.2f, %.2f, %.2f)", 
                     dist, cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z);
                
            if (dist < threshold) {
                target_set = false;
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.linear.z = 0.0;
                ROS_INFO("Target reached: (%.2f, %.2f, %.2f)", current_pos.x, current_pos.y, current_pos.z);
            }

            vel_pub.publish(cmd_vel);
        } else {
            ROS_WARN_THROTTLE(1.0, "No position data received yet in PID thread");
        }
        rate.sleep();
    }
    // Ensure velocity is zero when thread exits
    std::lock_guard<std::mutex> lock(data_mutex);
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    vel_pub.publish(cmd_vel);
    ROS_INFO("PID thread stopped");
}

int main(int argc, char** argv) {
    // Check for drone name argument
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " drone_name" << std::endl;
        return 1;
    }
    drone_name = argv[1];

    // Initialize ROS node
    ros::init(argc, argv, "drone_control_node");
    ros::NodeHandle nh;

    // Publisher for velocity commands
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Subscriber for Gazebo model states
    ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 10, modelStatesCallback);

    // Start callback thread
    std::thread callback_thread(callbackThread);

    ROS_INFO("Drone control node started for %s. Enter commands:", drone_name.c_str());
    ROS_INFO(" - 3 numbers: Target position (x y z)");
    ROS_INFO(" - 6 numbers: Velocity (vx vy vz wx wy wz)");
    ROS_INFO(" - 9 numbers: PID gains (kp_x ki_x kd_x kp_y ki_y kd_y kp_z ki_z kd_z)");

    std::thread pid_thread; // Thread object for PID control

    while (ros::ok()) {
        std::string input;
        if (std::getline(std::cin, input)) { // Blocking call for input
            // Stop any running PID thread
            stop_pid_thread = true;
            if (pid_thread.joinable()) {
                pid_thread.join();
            }
            stop_pid_thread = false;

            std::istringstream iss(input);
            std::vector<double> nums;
            double num;
            while (iss >> num) nums.push_back(num);

            std::lock_guard<std::mutex> lock(data_mutex);
            if (nums.size() == 3) { // Set target position and start PID thread
                target_pos.x = nums[0];
                target_pos.y = nums[1];
                target_pos.z = nums[2];
                target_set = true;
                ROS_INFO("Target set to: (%.2f, %.2f, %.2f)", target_pos.x, target_pos.y, target_pos.z);

                // Start PID thread
                pid_thread = std::thread(pidControlThread);
            } else if (nums.size() == 6) { // Direct velocity control
                cmd_vel.linear.x = nums[0];
                cmd_vel.linear.y = nums[1];
                cmd_vel.linear.z = nums[2];
                cmd_vel.angular.x = nums[3];
                cmd_vel.angular.y = nums[4];
                cmd_vel.angular.z = nums[5];
                target_set = false;
                ROS_INFO("Velocity set: (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f)",
                         cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z,
                         cmd_vel.angular.x, cmd_vel.angular.y, cmd_vel.angular.z);
                vel_pub.publish(cmd_vel);
            } else if (nums.size() == 9) { // Set PID gains
                pid_x.setGains(nums[0], nums[1], nums[2]);
                pid_y.setGains(nums[3], nums[4], nums[5]);
                pid_z.setGains(nums[6], nums[7], nums[8]);
                ROS_INFO("PID gains updated: x(%.2f, %.2f, %.2f), y(%.2f, %.2f, %.2f), z(%.2f, %.2f, %.2f)",
                         nums[0], nums[1], nums[2], nums[3], nums[4], nums[5], nums[6], nums[7], nums[8]);
            } else {
                ROS_ERROR("Invalid input: Use 3 numbers (position), 6 numbers (velocity), or 9 numbers (gains).");
            }
        }
    }

    // Cleanup
    shutdown = true;
    stop_pid_thread = true;
    if (pid_thread.joinable()) {
        pid_thread.join();
    }
    if (callback_thread.joinable()) {
        callback_thread.join();
    }
    return 0;
}


//source devel/setup.bash
//roslaunch hector_quadrotor_gazebo quadrotor_empty_world.launch
//rosservice call /gazebo/unpause_physics
//rosrun drone_control drone_control_node quadrotor
//killall gzserver gzclient 
//rosclean purge
//0.5 0.01 0.1 0.5 0.01 0.1 0.5 0.01 0.1
//source devel/setup.bash
//catkin_make
