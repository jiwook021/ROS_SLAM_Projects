#ifndef SERVICE_COMMON_H
#define SERVICE_COMMON_H

#include <string>
#include <unordered_map>
#include <functional>

namespace ROS {

struct AddTwoIntsRequest {
    int a, b;
};

struct AddTwoIntsResponse {
    int sum;
};

// Forward declaration of serviceServer
class serviceServer;

// serviceClient class declaration
class serviceClient {
public:
    serviceClient(const std::string& service_name);
    bool call(AddTwoIntsRequest& req, AddTwoIntsResponse& res);
    static void registerServer(const std::string& name, serviceServer* server);
    static void unregisterServer(const std::string& name);

private:
    std::string service_name_;
    static std::unordered_map<std::string, serviceServer*> servers_;
};

} // namespace ROS

#endif // SERVICE_COMMON_H