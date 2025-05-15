#include "influxdb/influxdb.hpp"

#include <sstream>
#include <fmt/core.h>

namespace influxDB {

    std::string createPayloadSingleVal(ValueT valueType, primaryKey_t primaryKey, long double value, std::chrono::nanoseconds timestamp) {
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
            default:                type = "MISC";
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

    std::string vectorToString(const std::vector<primaryKey_t>& vec) {
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < vec.size(); ++i) {
            oss << "\"" << vec[i] << "\"";
            if (i != vec.size() - 1) {
                oss << ",";
            }
        }
        oss << "]";
        return oss.str();
    }

    std::string createPayloadGetSingleValue(std::string bucket, Attribute attribute, std::vector<primaryKey_t> primaryKeys) {
        std::string attr;

        switch (attribute) {
            case CPU_UTILIZATION: attr = "CPU_UTILIZATION"; break;
            default: break;
        }


        return fmt::format(R"(from(bucket: "{}")
  |> range(start: -1s)
  |> filter(fn: (r) => r["_measurement"] == "{}")
  |> filter(fn: (r) => r["_field"] == "value")
  |> filter(fn: (r) => contains(value: r["primaryKey"], set: ["12"]))
  |> aggregateWindow(every: 1s, fn: mean, createEmpty: false)
  |> group()
  |> sum(column: "_value")
  |> yield(name: "sum_of_means")", bucket, attr, vectorToString(primaryKeys));
    }

//         return fmt::format(R"(from(bucket: "{}")
//   |> range(start: -1s)
//   |> filter(fn: (r) => r["_measurement"] == "{}")
//   |> filter(fn: (r) => r["_field"] == "value")
//   |> filter(fn: (r) => r["primaryKey"] == "{}")
//   |> aggregateWindow(every: 1s, fn: mean, createEmpty: false)
//   |> yield(name: "mean"))", bucket, attr, primaryKey);
//     }

    double extractValueFromCSV(const std::string& csv) {
        std::istringstream ss(csv);
        std::string line;

        std::getline(ss, line);

        while (std::getline(ss, line)) {
            if (line.empty() || line[0] == '#') continue;

            std::stringstream ls(line);
            std::string token;
            std::vector<std::string> tokens;

            while (std::getline(ls, token, ',')) {
                tokens.push_back(token);
            }

            if (tokens.size() > 6) {
                try {
                    return std::stod(tokens[6]);
                } catch (...) {
                    return -1.0;
                }
            }
        }

        return -1.0; // Kein Wert gefunden
    }

}