#pragma once

#include <string>

#include <ipc/sharedMem.hpp>

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
    std::string getPayload(std::string name, u_int64_t handle, sharedMem::State state, pid_t pid, time_t timestamp);
    std::string getPayloadSearch(std::string name);
    std::string getPayloadRequestByPrimaryKey(pid_t pid);
    std::string getPayloadSetNodeStateByPID(pid_t pid, time_t timestamp, sharedMem::State state);
    std::string getPayloadSetStateMachine(u_int64_t handle, u_int64_t stateMachine, sharedMem::State state);
    std::string getPayloadSetStateTransition(u_int64_t stateMachine, sharedMem::State state, time_t timestamp);
}