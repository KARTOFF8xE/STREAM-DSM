#pragma once

#include <string>


namespace actionservice {

    /**
     * @brief Builds the payload used to query neo4j.
     * 
     * @param name The name of the Action.
     * @param handle The Node handle the Action Server belongs to.
     * 
     * @returns The Payload for the Query.
     */
    std::string getPayload(std::string name, u_int64_t handle);

}