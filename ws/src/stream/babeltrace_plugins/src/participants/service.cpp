#include <fmt/core.h>
#include <string>
#include <iostream>

#include <nlohmann/json.hpp>

#include <neo4j/service/service.hpp>
#include <neo4j/actionservice/actionservice.hpp>
#include <influxdb/influxdb.hpp>
#include <curl/myCurl.hpp>
#include <ipc/util.hpp>

#include "interface.hpp"
#include "participants/service.hpp"

using json = nlohmann::json;


void Service::extractInfo(const bt_event *event) {
    const bt_field *payload_field = bt_event_borrow_payload_field_const(event);
    const bt_field_class *field_class = bt_field_borrow_class_const(payload_field);
    if (bt_field_class_get_type(field_class) == BT_FIELD_CLASS_TYPE_STRUCTURE) {
        const bt_field *field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "service_name");
        this->name = std::string(bt_field_string_get_value(field));
        field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "node_handle");
        this->node_handle = bt_field_integer_unsigned_get_value(field);

        if (this->name.find("/_action/") != std::string::npos) this->isAction = true;
        const std::string ext("/_action/send_goal");
        if (this->name != ext &&
            this->name.size() > ext.size() &&
            this->name.substr(this->name.size() - ext.size()) == "/_action/send_goal") {
                this->name = this->name.substr(0, this->name.size() - ext.size());
        }
    } else { printf("\033[33;1WRONG TYPE\033[0m\n"); }
}

std::string Service::getGraphPayload() {
    if (this->name.find("/_action/") != std::string::npos) return "";

    if (this->isAction) return actionservice::getPayload(this->name, this->node_handle);

    return service::getPayload(this->name, this->node_handle);
}

void Service::toGraph(std::string payload) {
    if (payload == "") return;

    std::string response = curl::push(payload, curl::NEO4J);

    json data = nlohmann::json::parse(response);
    json row = data["results"][0]["data"][0]["row"];

    if (!row.empty() && !row[0].empty() && !row[0][0]["node_id"].empty())   this->primaryKey = row[0][0]["node_id"];

    size_t counter = 0;
    while (!row.empty() &&
        !row[0].empty() &&
        !row[0][counter].empty() &&
        !row[0][counter]["client_id"].empty()
        ) {
        this->client_primaryKeys.push_back(row[0][counter]["client_id"]);
        counter++;
    }
}

std::string Service::getTimeSeriesPayload() {
    if (this->name.find("/_action/") != std::string::npos) return "";

    if (this->isAction) return influxDB::createPayloadSingleVal(
        influxDB::ValuePairs {
            .attribute  = influxDB::ACTIONSERVICE,
            .primaryKey = this->primaryKey,
            .timestamp  = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()),
            .value      = 1,
        }
    );

    return influxDB::createPayloadSingleVal(
        influxDB::ValuePairs {
            .attribute  = influxDB::SERVICE,
            .primaryKey = this->primaryKey,
            .timestamp  = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()),
            .value      = 1,
        }
    );
}

void Service::toTimeSeries(std::string payload) {
    curl::push(payload, curl::INFLUXDB_WRITE);
}

// void Service::response(Communication &communication) {
//     if (!this->isAction) {
//         for (primaryKey_t item : this->client_primaryKeys) {
//             NodeIsServerForUpdate msg {
//                 .primaryKey     = this->primaryKey,
//                 .clientNodeId   = item,
//                 .isUpdate       = true,
//             };
//             util::parseString(msg.srvName, this->name);
            
//             communication.server.sendNodeIsServerForUpdate(msg, communication.pid, false);
//         }
//     } else {
//         if (this->name.find("/_action/") != std::string::npos) return;

//         for (primaryKey_t item : this->client_primaryKeys) {
//             NodeIsActionServerForUpdate msg {
//                 .primaryKey         = this->primaryKey,
//                 .actionclientNodeId = item,
//                 .isUpdate           = true,
//             };
//             util::parseString(msg.srvName, this->name);

//             communication.server.sendNodeIsActionServerForUpdate(msg, communication.pid, false);
//         }
//     }
// }