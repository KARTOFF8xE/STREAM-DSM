#pragma once

#include <string>


namespace node {
    
    /**
     * @brief Builds the payload used to query neo4j.
     * 
     * @param name The name of the Node.
     * @param handle The Node handle.
     * @param pid The pid of the Nodes process.
     * 
     * @returns The Payload for the Query.
     */
    std::string getPayload(std::string name, u_int64_t handle, pid_t pid);
    std::string getPayloadRequestByPrimaryKey(pid_t pid);
    std::string getPayloadSetNodeOffline(pid_t pid);
}