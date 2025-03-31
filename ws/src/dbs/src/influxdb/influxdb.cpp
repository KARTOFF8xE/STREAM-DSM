#include "influxdb/influxdb.hpp"


namespace influxDB {

    std::string createPayloadSingleVal(ValueT valueType, primaryKey_t primaryKey, size_t value, std::chrono::nanoseconds timestamp) {
        std::string type;
        switch (valueType) {
            case CPU_UTILIZATION:   type = "CPU_UTILIZATION"; break;
            case STATECHANGE:       type = "STATECHANGE"; break;
            case CLIENT:            type = "CLIENT"; break;
            case ACTIONCLIENT:      type = "ACTIONCLIENT"; break;
            case SERVICE:           type = "SERVICE"; break;
            case ACTIONSERVICE:     type = "ACTIONSERVICE"; break;
            case PUBLISHER:         type = "PUBLISHER"; break;
            case SUBSCRIBER:        type = "SUBSCRIBER"; break;
            default:                type = "misc";
        }

        return type + "," +
            "primaryKey=" + std::to_string(primaryKey) + " " +
            "value=" + std::to_string(value) + " " +
            std::to_string(timestamp.count());
    }

    std::string createPayloadMultipleValSameTime(ValueT valueType, std::vector<ValuePairs> pairs, std::chrono::nanoseconds timestamp) {
        std::string type;
        switch (valueType) {
            case CPU_UTILIZATION:   type = "CPU_UTILIZATION"; break;
            case STATECHANGE:       type = "STATECHANGE"; break;
            case CLIENT:            type = "CLIENT"; break;
            case ACTIONCLIENT:      type = "ACTIONCLIENT"; break;
            case SERVICE:           type = "SERVICE"; break;
            case ACTIONSERVICE:     type = "ACTIONSERVICE"; break;
            case PUBLISHER:         type = "PUBLISHER"; break;
            case SUBSCRIBER:        type = "SUBSCRIBER"; break;
            default:                type = "misc";
        }

        std::string payload = "";

        for (auto pair : pairs) {
            payload += type + "," + "primaryKey=" + std::to_string(pair.primaryKey) + " value=" + std::to_string(pair.value) + "\n";
        }

        return payload;
    }

}