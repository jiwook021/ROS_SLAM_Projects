#include <iostream>
#include <string>
#include <functional>
#include <unordered_map>

namespace ROS {

// Placeholder structs (replace with your actual request/response types)
struct AddTwoIntsRequest {
    int a, b;
};
struct AddTwoIntsResponse {
    int sum;
};

// Forward declaration of serviceServer
class serviceServer;

// Define serviceClient
class serviceClient {
public:
    serviceClient(const std::string& service_name) : service_name_(service_name) {}

    // Declare the call method (defined later)
    bool call(AddTwoIntsRequest& req, AddTwoIntsResponse& res);

    // Static methods to manage servers
    static void registerServer(const std::string& name, serviceServer* server) {
        servers_[name] = server;
    }
    static void unregisterServer(const std::string& name) {
        servers_.erase(name);
    }

private:
    std::string service_name_;
    static std::unordered_map<std::string, serviceServer*> servers_;
};

// Initialize static member
std::unordered_map<std::string, serviceServer*> serviceClient::servers_;

// Define serviceServer
class serviceServer {
public:
    using CallbackType = std::function<bool(AddTwoIntsRequest&, AddTwoIntsResponse&)>;

    serviceServer(const std::string& service_name, CallbackType callback)
        : service_name_(service_name), callback_(callback) {
        serviceClient::registerServer(service_name_, this);
        std::cout << "Service '" << service_name_ << "' advertised." << std::endl;
    }

    ~serviceServer() {
        serviceClient::unregisterServer(service_name_);
        std::cout << "Service '" << service_name_ << "' shut down." << std::endl;
    }

    bool handleRequest(AddTwoIntsRequest& req, AddTwoIntsResponse& res) {
        return callback_(req, res);
    }

private:
    std::string service_name_;
    CallbackType callback_;
};

// Implement serviceClient::call
bool serviceClient::call(AddTwoIntsRequest& req, AddTwoIntsResponse& res) {
    auto it = servers_.find(service_name_);
    if (it == servers_.end()) {
        std::cerr << "Service '" << service_name_ << "' not found!" << std::endl;
        return false;
    }
    serviceServer* server = it->second;
    return server->handleRequest(req, res);
}

// advertise_service function
serviceServer advertise_service(const std::string& service_name,
                                bool (*callback)(AddTwoIntsRequest&, AddTwoIntsResponse&)) {
    return serviceServer(service_name, callback);
}

} // namespace ROS

// Sample callback function
bool addTwoInts(ROS::AddTwoIntsRequest& req, ROS::AddTwoIntsResponse& res) {
    res.sum = req.a + req.b;
    std::cout << "Request: " << req.a << " + " << req.b << " = " << res.sum << std::endl;
    return true;
}

// Main function for testing
int main() {
    ROS::serviceServer server = ROS::advertise_service("add_two_ints", addTwoInts);
    ROS::serviceClient client("add_two_ints");

    ROS::AddTwoIntsRequest req;
    ROS::AddTwoIntsResponse res;
    req.a = 5;
    req.b = 3;

    if (client.call(req, res)) {
        std::cout << "Service call succeeded: " << res.sum << std::endl;
    } else {
        std::cout << "Service call failed." << std::endl;
    }

    return 0;
}