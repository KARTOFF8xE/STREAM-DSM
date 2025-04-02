#pragma once

#include <string>


namespace publisher {

    /**
     * @brief Builds the payload used to query neo4j.
     * 
     * @param name The name of the Topic the publisher publishes to.
     * @param handle The Node handle the publisher belongs to.
     * 
     * @returns The Payload for the Query.
     */
    std::string getPayload(std::string name, u_int64_t handle);

}