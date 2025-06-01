#include "influxdb/influxdb.hpp"

#include <sstream>
#include <fmt/core.h>
#include <iomanip>
#include <curl/myCurl.hpp>

namespace influxDB {

    std::string createPayloadSingleVal(ValuePairs pair) {
        std::string type;
        switch (pair.attribute) {
            case CPU_UTILIZATION:   type = "CPU_UTILIZATION"; break;
            case STATECHANGE:       type = "STATECHANGE"; break;
            case PUBLISHINGRATE:    type = "PUBLISHINGRATE"; break;
            case CLIENT:            type = "CLIENT"; break;
            case ACTIONCLIENT:      type = "ACTIONCLIENT"; break;
            case SERVICE:           type = "SERVICE"; break;
            case ACTIONSERVICE:     type = "ACTIONSERVICE"; break;
            case PUBLISHER:         type = "PUBLISHER"; break;
            case SUBSCRIBER:        type = "SUBSCRIBER"; break;
            default:                type = "MISC";
        }

        return type + "," +
            "primaryKey=" + std::to_string(pair.primaryKey) + " value=" + std::to_string(pair.value) + "\n";
    }

    std::string createPayloadMultipleVal(std::vector<ValuePairs> pairs) {
        std::string type;
        
        std::string payload = "";
        for (auto pair : pairs) {
            switch (pair.attribute) {
                case CPU_UTILIZATION:   type = "CPU_UTILIZATION"; break;
                case STATECHANGE:       type = "STATECHANGE"; break;
                case PUBLISHINGRATE:    type = "PUBLISHINGRATE"; break;
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
            case PUBLISHINGRATES: attr = "PUBLISHINGRATE"; break;
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

    std::string makeFluxStringList(const std::vector<primaryKey_t>& items) {
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < items.size(); ++i) {
            oss << "\"" << items[i] << "\"";
            if (i < items.size() - 1) {
                oss << ",";
            }
        }
        oss << "]";
        return oss.str();
    }

    std::string jsonEscape(const std::string& s) {
        std::ostringstream o;
        for (char c : s) {
            switch (c) {
                case '\"': o << "\\\""; break;
                case '\\': o << "\\\\"; break;
                case '\n': o << "\\n"; break;
                case '\r': o << "\\r"; break;
                case '\t': o << "\\t"; break;
                default:
                    if (static_cast<unsigned char>(c) < 0x20) {
                        o << "\\u"
                        << std::hex << std::uppercase
                        << (int)c;
                    } else {
                        o << c;
                    }
            }
        }
        return o.str();
    }

    std::string createPayloadForTask(std::string bucket, std::vector<primaryKey_t> primaryKeys, primaryKey_t destPrimaryKey) {
        std::string fluxScript = fmt::format(R"(
        option task = {{name: "{}_in", every: 1s}}
        pkList = {}
        from(bucket: "{}")
        |> range(start: -task.every)
        |> filter(fn: (r) => r._measurement == "PUBLISHINGRATE" and r._field == "value" and contains(value: r.primaryKey, set: pkList))
        |> aggregateWindow(every: 1s, fn: mean, createEmpty: false)
        |> group(columns: ["_time"])
        |> sum()
        |> map(fn: (r) => ({{
            _time: r._time,
            _value: r._value,
            _field: "value",
            _measurement: "PUBLISHINGRATE",
            primaryKey: "{}"
        }}))
        |> to(bucket: "{}", org: "TUBAF")
        )", destPrimaryKey, makeFluxStringList(primaryKeys), bucket, destPrimaryKey, bucket);

        std::string escapedFlux = jsonEscape(fluxScript);

        return std::string("{") +
            "\"org\": \"TUBAF\"," +
            "\"flux\": \"" + escapedFlux + "\"" +
            "}";
    }

    std::string getTaskIDByName(const std::string& taskName) {
        std::string response = curl::getTaskIdsForNames();

        size_t pos = 0;
        while ((pos = response.find("\"name\":\"" + taskName + "\"", pos)) != std::string::npos) {
            size_t idPos = response.rfind("\"id\":\"", pos);
            if (idPos == std::string::npos) return "";

            idPos += 6;
            size_t idEnd = response.find("\"", idPos);
            if (idEnd == std::string::npos) return "";

            return response.substr(idPos, idEnd - idPos);
        }

        return "";
    }

}