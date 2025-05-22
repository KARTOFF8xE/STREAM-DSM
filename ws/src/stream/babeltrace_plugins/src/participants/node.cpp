#include <fmt/core.h>
#include <string>
#include <iostream>

#include <nlohmann/json.hpp>

#include <neo4j/node/node.hpp>
#include <influxdb/influxdb.hpp>
#include <curl/myCurl.hpp>
#include <ipc/common.hpp>
#include <ipc/util.hpp>

#include "common.hpp"
#include "interface.hpp"
#include "participants/node.hpp"


void Node::extractInfo(const bt_event *event) {
    const bt_field *payload_field = bt_event_borrow_payload_field_const(event);
    const bt_field_class *field_class = bt_field_borrow_class_const(payload_field);
    if (bt_field_class_get_type(field_class) == BT_FIELD_CLASS_TYPE_STRUCTURE) {
            const bt_field *field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "node_name");
            this->name = std::string(bt_field_string_get_value(field));
            field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "namespace");
            this->nameSpace = std::string(bt_field_string_get_value(field));
            field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "node_handle");
            this->handle = bt_field_integer_unsigned_get_value(field);
    } else { printf("\033[33;1WRONG TYPE\033[0m\n"); }

    const bt_field *ctx_field = bt_event_borrow_common_context_field_const(event);
    field_class = bt_field_borrow_class_const(ctx_field);
    if (bt_field_class_get_type(field_class) == BT_FIELD_CLASS_TYPE_STRUCTURE) {
        const bt_field *field = bt_field_structure_borrow_member_field_by_name_const(ctx_field, "vpid");
        this->pid = uint32_t(bt_field_integer_signed_get_value(field));
    } else { printf("\033[33;1WRONG TYPE\033[0m\n"); }

    return;
}

std::string Node::getFullName() {
    if (this->nameSpace == "/") {
        return this->nameSpace + this->name;
    }

    return this->nameSpace + "/" + this->name;
}

std::string Node::getGraphPayload() {
    return node::getPayload(getFullName(), this->handle, this->pid);
}

void Node::toGraph(std::string payload) {
    std::string response = curl::push(payload, curl::NEO4J);

    nlohmann::json data = nlohmann::json::parse(response);
    if (!data["results"].empty() &&
        !data["results"][0]["data"].empty() && 
        !data["results"][0]["data"][0]["meta"].empty())
        {
        this->primaryKey = static_cast<primaryKey_t>(data["results"][0]["data"][0]["meta"][0]["id"]);
    } else {
        std::cout << "Failed parsing JSON (wanted ID)" << std::endl;
    }
    if (!data["results"].empty() &&
        !data["results"][0]["data"].empty() && 
        !data["results"][0]["data"][0]["row"].empty())
        {
        this->bootcounter       = data["results"][0]["data"][0]["row"][0]["bootcounter"];
        this->stateChangeTime   = data["results"][0]["data"][0]["row"][0]["stateChangeTime"];
    } else {
        std::cout << "Failed parsing JSON (wanted ID)" << std::endl;
    }
}

std::string Node::getTimeSeriesPayload() {
    return influxDB::createPayloadSingleVal(
        influxDB::ValuePairs {
            .attribute  = influxDB::STATECHANGE,
            .primaryKey = this->primaryKey,
            .timestamp  = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()),
            .value      = 1,
        }
    );
}

void Node::toTimeSeries(std::string payload) {
    std::string response = curl::push(payload, curl::INFLUXDB_WRITE);
}

// void Node::response(Communication &communication) {
//     NodeResponse msg {
//         .primaryKey = this->primaryKey,
//         .alive = true,
//         .aliveChangeTime = this->stateChangeTime,
//         .bootCount = this->bootcounter,
//         .pid = (pid_t) this->pid,
//     };
//     util::parseString(msg.name, this->getFullName());

//     communication.server.sendNodeResponse(msg, communication.pid, false);
// }