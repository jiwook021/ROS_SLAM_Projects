#include "service_common.h"
#include <iostream>

namespace ROS {

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

serviceServer advertise_service(const std::string& service_name,
                               bool (*callback)(AddTwoIntsRequest&, AddTwoIntsResponse&)) {
    return serviceServer(service_name, callback);
}

} // namespace ROS