// Include standard C++ libraries for basic functionality
#include <iostream>         // For printing to the console (e.g., std::cout)
#include <string>           // For working with strings (e.g., std::string)
#include <vector>           // For storing a list of messages in a queue
#include <chrono>           // For timing and sleeping (e.g., milliseconds)
#include <thread>           // For creating threads and pausing execution (e.g., sleep_for)
#include <csignal>          // For handling signals like Ctrl+C
#include <functional>       // For std::ref to pass references to threads

// Define a simple structure to represent a "message"
// This mimics the std_msgs::String type used in real ROS
struct Message {
    std::string data;       // The message content, a string like "Hello ROS World"
};

// A class to simulate a ROS subscriber
// This pretends to listen to a topic and process messages
class FakeSubscriber {
public:
    // Constructor: Sets up the subscriber with a topic name and a callback function
    // 'topic' is the name of the channel (e.g., "chatter"), 'callback' is the function to call
    FakeSubscriber(const std::string& topic, void (*callback)(const Message&))
        : topic_(topic),          // Initialize the topic name
          callback_(callback)     // Store the callback function pointer
    {}

    // Method to add a message to the queue
    // In real ROS, this would come from a publisher over the network
    void addMessage(const std::string& data) {
        messages_.push_back({data});  // Add a new message with the given data to the queue
    }

    // Method to process one message from the queue
    // Returns true if a message was processed, false if the queue is empty
    bool processOne() {
        if (!messages_.empty()) {         // Check if there are any messages in the queue
            Message msg = messages_.front();  // Get the first message in the queue
            messages_.erase(messages_.begin());  // Remove that message from the queue
            callback_(msg);               // Call the callback function with the message
            return true;                  // Indicate we processed a message
        }
        return false;                     // No messages to process
    }

private:
    std::string topic_;                   // The name of the topic (e.g., "chatter")
    void (*callback_)(const Message&);    // Pointer to the callback function
    std::vector<Message> messages_;       // Queue to hold incoming messages
};

// Global variable to control whether the node keeps running
// 'volatile' ensures it’s updated correctly across threads or signals
volatile bool keepRunning = true;

// Signal handler for Ctrl+C (SIGINT)
// This function runs when you press Ctrl+C to stop the program
void signalHandler(int signum) {
    keepRunning = false;    // Set keepRunning to false to stop the spin loop
    std::cout << "Ctrl+C detected, shutting down...\n";  // Notify user of shutdown
}

// Simulated version of ros::ok()
// In real ROS, this checks if the node is still active (e.g., no shutdown signal)
bool fakeRosOk() {
    return keepRunning;     // Return true to keep running, false to stop
}

// Our simplified version of ros::spin()
// This keeps the node alive, processing messages until shutdown
void fakeRosSpin(FakeSubscriber& sub) {
    while (fakeRosOk()) {       // Loop as long as the node should keep running
        if (!sub.processOne()) {  // Try to process one message; if none, sleep
            // No messages in the queue, so pause briefly to avoid using too much CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(4000));
        }
        // In real ROS, this loop would also check network events, timers, etc.
    }
    std::cout << "Node shutting down\n";  // Print a message when the loop exits
}

// Callback function to handle incoming messages
// This is like chatterCallback in the ROS listener node
void myCallback(const Message& msg) {
    std::cout << "I heard: [" << msg.data << "]\n";  // Print the message content
}

// Function to add messages in a separate thread
// Simulates a publisher adding messages to the queue
void addMessageFunction(FakeSubscriber& sub) {
    for (int i = 10; i < 120; i++) {  // Loop from 10 to 119
        sub.addMessage("Hello ROS World");  // Add the same message each time
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Wait 0.1 second
        if (!fakeRosOk()) break;  // Stop if shutdown is requested
    }
}

// Main function: Entry point of the program
int main() {
    // Set up the signal handler for Ctrl+C
    signal(SIGINT, signalHandler);  // When Ctrl+C is pressed, call signalHandler

    // Print a startup message to let the user know the node is starting
    std::cout << "Starting fake listener node\n";

    // Create a fake subscriber for the "chatter" topic with myCallback
    FakeSubscriber sub("chatter", myCallback);

    // Simulate adding initial messages to the queue
    // In real ROS, these would come from a publisher like talker.cpp
    sub.addMessage("Hello ROS World 0");  // Add first test message
    sub.addMessage("Hello ROS World 1");  // Add second test message

    // Notify the user that we’re entering the spin loop
    std::cout << "Entering spin loop\n";

    // Create a thread to run addMessageFunction, passing 'sub' by reference
    std::thread t1(addMessageFunction, std::ref(sub));  // Use std::ref to avoid copying

    // Call our fake ros::spin() to process messages in the main thread
    fakeRosSpin(sub);

    // Wait for the thread to finish after spin exits (e.g., after Ctrl+C)
    t1.join();

    // This line only runs after fakeRosSpin exits
    std::cout << "Done\n";

    return 0;  // Exit the program cleanly
}