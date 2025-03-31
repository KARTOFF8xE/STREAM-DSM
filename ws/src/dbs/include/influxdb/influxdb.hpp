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

    // TODO
    std::string createPayloadSingleVal(ValueT valueType, primaryKey_t primaryKey, size_t value, std::chrono::nanoseconds timestamp = std::chrono::high_resolution_clock::now().time_since_epoch());

    // TODO
    std::string createPayloadMultipleValSameTime(ValueT valueType, std::vector<ValuePairs> pairs, std::chrono::nanoseconds timestamp = std::chrono::high_resolution_clock::now().time_since_epoch());

}