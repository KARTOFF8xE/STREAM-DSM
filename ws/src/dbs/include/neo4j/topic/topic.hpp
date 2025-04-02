#pragma once

#include <string>


namespace topic {

    /**
     * @brief Builds the payload used to query neo4j.
     * 
     * @param pid The primaryKey of the Topic.
     * 
     * Sorry I know the name pid is confusing, will fix it. // TODO
     * 
     * @returns The Payload for the Query.
     */
    std::string getPayloadRequestByPrimaryKey(pid_t pid);

}