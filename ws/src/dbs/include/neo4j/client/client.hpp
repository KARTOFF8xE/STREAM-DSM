#pragma once

#include <string>


namespace client {

    /**
     * @brief Builds the payload used to query neo4j.
     * 
     * @param serviceName The name of the Service.
     * @param handle The Node handle the Client belongs to.
     * 
     * @returns The Payload for the Query.
     */
    std::string getPayload(std::string serviceName, u_int64_t nodeHandle);
    // TODO
    std::string getPayload(std::string serviceName, u_int64_t nodeHandle, std::string actionName);

}