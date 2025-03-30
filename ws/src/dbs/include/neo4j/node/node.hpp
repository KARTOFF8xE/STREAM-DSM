#pragma once

#include <string>


namespace node {

    std::string getPayload(std::string name, u_int64_t handle, pid_t pid);
    std::string getPayloadRequestByPrimaryKey(pid_t pid);
    std::string getPayloadSetNodeOffline(pid_t pid);
}