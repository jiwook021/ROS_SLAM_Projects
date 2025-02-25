#include "service_common.h"
#include <iostream>

bool addTwoInts(ROS::AddTwoIntsRequest& req, ROS::AddTwoIntsResponse& res) {
    res.sum = req.a + req.b;
    std::cout << "Request: " << req.a << " + " << req.b << " = " << res.sum << std::endl;
    return true;
}

int main() {
    // Create and advertise the server
    ROS::serviceServer server = ROS::advertise_service("add_two_ints", addTwoInts);
    
    // Create the client
    ROS::serviceClient client("add_two_ints");
    
    // Prepare and make a service call
    ROS::AddTwoIntsRequest req;
    req.a = 5;
    req.b = 3;
    ROS::AddTwoIntsResponse res;
    
    if (client.call(req, res)) {
        std::cout << "Service call succeeded: " << res.sum << std::endl;
    } else {
        std::cout << "Service call failed." << std::endl;
    }
    
    return 0;
}