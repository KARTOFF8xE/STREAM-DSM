#include "influxdb/influxdb.hpp"

#include <sstream>
#include <fmt/core.h>

namespace influxDB {

    std::string createPayloadSingleVal(ValuePairs pair) {
        std::string type;
        switch (pair.attribute) {
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
            "primaryKey=" + std::to_string(pair.primaryKey) + " " +
            "value=" + std::to_string(pair.value) + " " +
            std::to_string(pair.timestamp);
    }

    std::string createPayloadMultipleValSameTime(std::vector<ValuePairs> pairs) {
        std::string type;
        
        std::string payload = "";
        for (auto pair : pairs) {
            switch (pair.attribute) {
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

    std::string createPayloadGetSingleValue(std::string bucket, AttributeName attribute, std::vector<primaryKey_t> primaryKeys) {
        std::string attr;

        switch (attribute) {
            case CPU_UTILIZATION: attr = "CPU_UTILIZATION"; break;
            default: break;
        }


        return fmt::format(R"(from(bucket: "{}")
  |> range(start: -1s)
  |> filter(fn: (r) => r["_measurement"] == "{}")
  |> filter(fn: (r) => r["_field"] == "value")
  |> filter(fn: (r) => contains(value: r["primaryKey"], set: {}))
  |> aggregateWindow(every: 1s, fn: mean, createEmpty: false)
  |> group()
  |> sum(column: "_value")
  |> yield(name: "sum_of_means")
)", bucket, attr, vectorToString(primaryKeys));
    }


    double extractValueFromCSV(const std::string& csv) {
        std::istringstream stream(csv);
        std::string line;

        std::getline(stream, line);

        if (std::getline(stream, line)) {
            std::istringstream linestream(line);
            std::string cell;
            int columnIndex = 0;
            double value = 0.0;

            while (std::getline(linestream, cell, ',')) {
                if (columnIndex == 3) {
                    value = std::stod(cell);
                    break;
                }
                columnIndex++;
            }
            return value;
        }

        return -1;
    }

}