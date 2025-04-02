#pragma once

#include <chrono>
#include <string>
#include <vector>

#include <ipc/common.hpp>


namespace influxDB {

    enum ValueT {
        CPU_UTILIZATION,
        STATECHANGE,
        CLIENT,
        ACTIONCLIENT,
        SERVICE,
        ACTIONSERVICE,
        PUBLISHER,
        SUBSCRIBER,
    };

    struct ValuePairs {
        primaryKey_t    primaryKey;
        double          value;    
    };

    /**
     * @brief Builds the Payload to query Influx-DB, consisting of a single information.
     * 
     * @param valueType The type of value that will be pushed.
     * @param primaryKey The primaryKey of the entity the value belongs to.
     * @param value The value that is to be pushed.
     * @param timestamp The timestamp (by standard its the timestamp from the function call)
     * 
     * @returns The payload for the Query.
     */
    std::string createPayloadSingleVal(ValueT valueType, primaryKey_t primaryKey, size_t value, std::chrono::nanoseconds timestamp = std::chrono::high_resolution_clock::now().time_since_epoch());

    /**
     * @brief Builds the Payload to query Influx-DB, consisting of a multiple informations.
     * 
     * @param valueType The type of value that will be pushed.
     * @param pairs The pairs (primaryKey, value) that have to be pushed, all belonging to the same Type and have the same timestamp.
     * @param timestamp The timestamp (by standard its the timestamp from the function call)
     * 
     * @returns The payload for the Query.
     */
    std::string createPayloadMultipleValSameTime(ValueT valueType, std::vector<ValuePairs> pairs, std::chrono::nanoseconds timestamp = std::chrono::high_resolution_clock::now().time_since_epoch());

}