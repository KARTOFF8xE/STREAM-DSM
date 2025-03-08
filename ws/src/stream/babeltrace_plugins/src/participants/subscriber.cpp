#include <fmt/core.h>
#include <string>
#include <iostream>

#include <nlohmann/json.hpp>

#include <neo4j/subscriber/subscriber.hpp>
#include <curl/myCurl.hpp>

#include "interface.hpp"
#include "participants/subscriber.hpp"

using json = nlohmann::json;


void Subscriber::extractInfo(const bt_event *event) {
    const bt_field *payload_field = bt_event_borrow_payload_field_const(event);
    const bt_field_class *field_class = bt_field_borrow_class_const(payload_field);
    if (bt_field_class_get_type(field_class) == BT_FIELD_CLASS_TYPE_STRUCTURE) {
            const bt_field *field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "topic_name");
            this->name = std::string(bt_field_string_get_value(field));
            field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "node_handle");
            this->node_handle = bt_field_integer_unsigned_get_value(field);
            field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "subscription_handle");
    } else { printf("\033[33;1WRONG TYPE\033[0m\n"); }
}

std::string Subscriber::getPayload() {
    if (this->name.find("/_action/") != std::string::npos) return "";

    return subscriber::getPayload(this->name, this->node_handle);
}

void Subscriber::toGraph(std::string payload) {
    if (this->name.find("/_action/") != std::string::npos) return;

    std::string response = curl::push(payload);

    json data = nlohmann::json::parse(response);
    json row = data["results"][0]["data"][0]["row"];
    if (!row.empty() && !row[0]["node_id"].empty())     this->node_primaryKey = row[0]["node_id"];
    if (!row.empty() && !row[0]["topic_id"].empty())    this->primaryKey = row[0]["topic_id"];
}

void Subscriber::response(Communication &communication) {
    if (this->name.find("/_action/") != std::string::npos) return;

    NodeSubscribersToUpdate msg {
        .primaryKey     = this->node_primaryKey,
        .subscribesTo   = this->primaryKey,
        .isUpdate       = true,
    };

    communication.server.sendNodeSubscribersToUpdate(msg, communication.pid, false);
}