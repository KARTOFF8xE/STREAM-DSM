#pragma once

#include <chrono>
#include <string>
#include <vector>

#include <ipc/common.hpp>


namespace influxDB {

    enum ValueT {
        CPU_UTILIZATION,
        DISK_UTILIZATION,
        PUBLISHINGRATE,
        STATECHANGE,
        CLIENT,
        ACTIONCLIENT,
        SERVICE,
        ACTIONSERVICE,
        PUBLISHER,
        SUBSCRIBER,
    };

    struct ValuePairs {
        ValueT          attribute;
        primaryKey_t    primaryKey;
        time_t          timestamp;
        long double     value;
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
    std::string createPayloadSingleVal(ValuePairs valuePair);

    /**
     * @brief Builds the Payload to query Influx-DB, consisting of a multiple informations.
     * 
     * @param valueType The type of value that will be pushed.
     * @param pairs The pairs (primaryKey, value) that have to be pushed, all belonging to the same Type and have the same timestamp.
     * @param timestamp The timestamp (by standard its the timestamp from the function call)
     * 
     * @returns The payload for the Query.
     */
    std::string createPayloadMultipleVal(std::vector<ValuePairs> pairs);

    // TODO
    std::string createPayloadGetSingleValue(std::string bucket, AttributeName attribute, std::vector<primaryKey_t> primaryKeys);

    // TODO
    double extractValueFromCSV(const std::string& csv);

}