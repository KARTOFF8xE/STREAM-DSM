#pragma once

#include <string>


namespace service {

    /**
     * @brief Builds the payload used to query neo4j.
     * 
     * @param name The name of the Service.
     * @param handle The Node handle the Server belongs to.
     * 
     * @returns The Payload for the Query.
     */
    std::string getPayload(std::string name, u_int64_t handle);

}