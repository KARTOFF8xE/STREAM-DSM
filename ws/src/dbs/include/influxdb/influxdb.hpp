#pragma once

#include <chrono>
#include <string>

#include <ipc/common.hpp>


namespace influxDB {

    enum ValueT {
        cpuUtilization
    };
    
    std::string createPayload(ValueT valueType, primaryKey_t primaryKey, size_t value, std::chrono::nanoseconds = std::chrono::high_resolution_clock::now().time_since_epoch());

}