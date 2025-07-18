#pragma once

#include <string>


namespace curl {

    enum Destination {
        NEO4J,
        INFLUXDB_WRITE,
        INFLUXDB_READ,
        INFLUXDB_SETTASK,
        INFLUXDB_UPDATETASK,
        INFLUXDB_DELETETASK
    };
    
// const Destination neo4j {
//     .username = "neo4j",
//     .password = "123456789",
//     .url = "http://172.17.0.1:7474/db/neo4j/tx/commit"
// };

// const Destination influxDB {
//     .username = "",
//     .password = "WVvSEEbHPqeMFpcgWqThaEcU6u6SWJ-L26ct4oRuEJmKdMOk-ZG8XlKA5xcitJXENa2r2YNLNwxjE6-KKkx8xw==",
//     .url = "http://172.17.0.1:8086/api/v2/write?org=TUBAF&bucket=STREAM&precision=ns"
// };

/**
 * @brief Sends a payload to the specified destination using a POST request.
 *
 * @param payload A string containing the data to be sent in the POST request.
 * @param destination Specifies the destination server (either NEO4J or INFLUXDB_WRITE) to send the request to.
 *
 * @return The response from the server, if the request was successful.
 *         If there was an error, the response is empty.
 */
std::string push(std::string payload, const Destination destination);
std::string push(std::string payload, const Destination destination, std::string taskId);
std::string push(const Destination destination, std::string taskId);
std::string getTaskIdsForNames();
}