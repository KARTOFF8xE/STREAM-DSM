#pragma once

#include <string>


struct Property {
    std::string username;
    std::string password;
    std::string url;
};

namespace curl {

const Property neo4j {
    .username = "neo4j",
    .password = "123456789",
    .url = "http://172.17.0.1:7474/db/neo4j/tx/commit"
};

const Property influxDB {
    .username = "",
    .password = "WVvSEEbHPqeMFpcgWqThaEcU6u6SWJ-L26ct4oRuEJmKdMOk-ZG8XlKA5xcitJXENa2r2YNLNwxjE6-KKkx8xw=",
    .url = "http://172.17.0.1:8086/api/v2/write?org=TUBAF&bucket=STREAM&precision=ns"
};

std::string push(std::string payload, const Property property);

}