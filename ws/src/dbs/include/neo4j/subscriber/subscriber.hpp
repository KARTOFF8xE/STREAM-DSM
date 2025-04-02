#pragma once

#include <string>


namespace subscriber {

    /**
     * @brief Builds the payload used to query neo4j.
     * 
     * @param name The name of the Topic the subscriber subscribes to.
     * @param handle The Node handle the subscriber belongs to.
     * 
     * @returns The Payload for the Query.
     */
    std::string getPayload(std::string name, u_int64_t handle);

}