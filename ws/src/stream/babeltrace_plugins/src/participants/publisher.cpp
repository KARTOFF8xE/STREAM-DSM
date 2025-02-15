#include <fmt/core.h>
#include <string>
#include <iostream>

#include <neo4j/publisher/publisher.hpp>
#include <curl/myCurl.hpp>

#include "interface.hpp"
#include "participants/publisher.hpp"


void Publisher::extractInfo(const bt_event *event) {
    const bt_field *payload_field = bt_event_borrow_payload_field_const(event);
    const bt_field_class *field_class = bt_field_borrow_class_const(payload_field);
    if (bt_field_class_get_type(field_class) == BT_FIELD_CLASS_TYPE_STRUCTURE) {
            const bt_field *field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "topic_name");
            this->name = std::string(bt_field_string_get_value(field));
            field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "node_handle");
            this->node_handle = bt_field_integer_unsigned_get_value(field);
            field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "publisher_handle");
            this->pubsub_handle = bt_field_integer_unsigned_get_value(field);
    } else { printf("\033[33;1WRONG TYPE\033[0m\n"); }
}

std::string Publisher::getPayload() {
    return publisher::getPayload(this->name, this->pubsub_handle, this->node_handle);
}

void Publisher::toGraph(std::string payload) {
    curl::push(payload);
}