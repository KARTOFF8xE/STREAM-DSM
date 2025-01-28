#pragma once

#include <babeltrace2/babeltrace.h>

#include "neo4j.cpp"

Node extractNodeInfoFromEvent(const bt_event *event) {
    Node node;

    const bt_field *payload_field = bt_event_borrow_payload_field_const(event);
    const bt_field_class *field_class = bt_field_borrow_class_const(payload_field);
    if (bt_field_class_get_type(field_class) == BT_FIELD_CLASS_TYPE_STRUCTURE) {
            const bt_field *field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "node_name");
            node.name = std::string(bt_field_string_get_value(field));
            field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "namespace");
            node.nameSpace = bt_field_string_get_value(field);
            field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "node_handle");
            node.handle = bt_field_integer_unsigned_get_value(field);
    } else { printf("\033[33;1WRONG TYPE\033[0m\n"); }

    const bt_field *ctx_field = bt_event_borrow_common_context_field_const(event);
    field_class = bt_field_borrow_class_const(ctx_field);
    if (bt_field_class_get_type(field_class) == BT_FIELD_CLASS_TYPE_STRUCTURE) {
        const bt_field *field = bt_field_structure_borrow_member_field_by_name_const(ctx_field, "vpid");
        node.pid = uint32_t(bt_field_integer_signed_get_value(field));
    } else { printf("\033[33;1WRONG TYPE\033[0m\n"); }

    return node;
}

Topic extractTopicInfoFromEvent(const bt_event *event) {
    Topic topic;

    const bt_field *payload_field = bt_event_borrow_payload_field_const(event);
    const bt_field_class *field_class = bt_field_borrow_class_const(payload_field);
    if (bt_field_class_get_type(field_class) == BT_FIELD_CLASS_TYPE_STRUCTURE) {
            const bt_field *field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "topic_name");
            topic.name = std::string(bt_field_string_get_value(field));
            field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "node_handle");
            topic.node_handle = bt_field_integer_unsigned_get_value(field);
            field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "publisher_handle");
            if (field != NULL) {
                field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "publisher_handle");
                topic.pubsub_handle = bt_field_integer_unsigned_get_value(field);
            } else {
                field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "subscription_handle");
                topic.pubsub_handle = bt_field_integer_unsigned_get_value(field);
            }
    } else { printf("\033[33;1WRONG TYPE\033[0m\n"); }

    return topic;
}