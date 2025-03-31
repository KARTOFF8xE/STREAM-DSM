#include "influxdb/influxdb.hpp"


namespace influxDB {

    std::string createPayload(ValueT valueType, primaryKey_t primaryKey, size_t value, std::chrono::nanoseconds timestamp) {
        // return "temperature,sensor=room1 value=" + std::to_string(count) + " " + std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
        std::string type;
        switch (valueType) {
            case cpuUtilization: type = "cpuUtilization"; break;
            default: type = "misc";
        }

        return type + "," +
            "primaryKey=" + std::to_string(primaryKey) + " " +
            "value=" + std::to_string(value) + " " +
            std::to_string(timestamp.count());
    }

}