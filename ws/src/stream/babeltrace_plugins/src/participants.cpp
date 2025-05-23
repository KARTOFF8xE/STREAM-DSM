#include "participants.hpp"

#include <ipc/util.hpp>

#include <string_view>


sharedMem::TraceMessage extractNodeInfo(const bt_event *event) {
    sharedMem::TraceMessage msg(sharedMem::MessageType::NODETRACE);

    const bt_field *payload_field = bt_event_borrow_payload_field_const(event);
    const bt_field_class *field_class = bt_field_borrow_class_const(payload_field);
    if (bt_field_class_get_type(field_class) == BT_FIELD_CLASS_TYPE_STRUCTURE) {
            const bt_field *field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "node_name");
            util::parseString(msg.node.name, bt_field_string_get_value(field));
            field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "namespace");
            util::parseString(msg.node.nspace, bt_field_string_get_value(field));
            field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "node_handle");
            msg.node.handle = bt_field_integer_unsigned_get_value(field);
    } else { printf("\033[33;1WRONG TYPE\033[0m\n"); }

    const bt_field *ctx_field = bt_event_borrow_common_context_field_const(event);
    field_class = bt_field_borrow_class_const(ctx_field);
    if (bt_field_class_get_type(field_class) == BT_FIELD_CLASS_TYPE_STRUCTURE) {
        const bt_field *field = bt_field_structure_borrow_member_field_by_name_const(ctx_field, "vpid");
        msg.node.pid = pid_t(bt_field_integer_signed_get_value(field));
    } else { printf("\033[33;1WRONG TYPE\033[0m\n"); }

    return msg;
}

sharedMem::TraceMessage extractPublisherInfo(const bt_event *event) {
    sharedMem::TraceMessage msg(sharedMem::MessageType::PUBLISHERTRACE);

    const bt_field *payload_field = bt_event_borrow_payload_field_const(event);
    const bt_field_class *field_class = bt_field_borrow_class_const(payload_field);
    if (bt_field_class_get_type(field_class) == BT_FIELD_CLASS_TYPE_STRUCTURE) {
            const bt_field *field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "topic_name");
            util::parseString(msg.publisher.topicName, bt_field_string_get_value(field));
            field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "node_handle");
            msg.publisher.nodeHandle = bt_field_integer_unsigned_get_value(field);
            field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "publisher_handle");
    } else { printf("\033[33;1WRONG TYPE\033[0m\n"); }

    return msg;
}

sharedMem::TraceMessage extractSubscriberInfo(const bt_event *event) {
    sharedMem::TraceMessage msg(sharedMem::MessageType::SUBSCRIBERTRACE);

    const bt_field *payload_field = bt_event_borrow_payload_field_const(event);
    const bt_field_class *field_class = bt_field_borrow_class_const(payload_field);
    if (bt_field_class_get_type(field_class) == BT_FIELD_CLASS_TYPE_STRUCTURE) {
            const bt_field *field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "topic_name");
            util::parseString(msg.subscriber.topicName, bt_field_string_get_value(field));
            field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "node_handle");
            msg.subscriber.nodeHandle = bt_field_integer_unsigned_get_value(field);
            field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "subscription_handle");
    } else { printf("\033[33;1WRONG TYPE\033[0m\n"); }

    return msg;
}

sharedMem::TraceMessage extractServiceInfo(const bt_event *event) {
    sharedMem::TraceMessage msg(sharedMem::MessageType::SERVICETRACE);

    const bt_field *payload_field = bt_event_borrow_payload_field_const(event);
    const bt_field_class *field_class = bt_field_borrow_class_const(payload_field);
    if (bt_field_class_get_type(field_class) == BT_FIELD_CLASS_TYPE_STRUCTURE) {
        const bt_field *field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "service_name");
        util::parseString(msg.service.name, bt_field_string_get_value(field));
        field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "node_handle");
        msg.service.nodeHandle = bt_field_integer_unsigned_get_value(field);
    } else { printf("\033[33;1WRONG TYPE\033[0m\n"); }

    return msg;
}

sharedMem::TraceMessage extractClientInfo(const bt_event *event) {
    sharedMem::TraceMessage msg(sharedMem::MessageType::CLIENTTRACE);

    const bt_field *payload_field = bt_event_borrow_payload_field_const(event);
    const bt_field_class *field_class = bt_field_borrow_class_const(payload_field);
    if (bt_field_class_get_type(field_class) == BT_FIELD_CLASS_TYPE_STRUCTURE) {
        const bt_field *field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "service_name");
        util::parseString(msg.service.name, bt_field_string_get_value(field));
        field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "node_handle");
        msg.service.nodeHandle = bt_field_integer_unsigned_get_value(field);
    } else { printf("\033[33;1WRONG TYPE\033[0m\n"); }

    return msg;
}


sharedMem::TraceMessage extractTracedMessage(const bt_message *message) {
    const bt_event *event = bt_message_event_borrow_event_const(message);
    const bt_event_class *eventClass = bt_event_borrow_class_const(event);

    std::string_view eventName(bt_event_class_get_name(eventClass));

    using namespace std::string_view_literals;
    if (eventName == "ros2:rcl_node_init"sv)            return extractNodeInfo(event);
    if (eventName == "ros2:rcl_publisher_init"sv)       return extractPublisherInfo(event);
    if (eventName == "ros2:rcl_subscription_init"sv)    return extractSubscriberInfo(event);
    if (eventName == "ros2:rcl_service_init"sv)         return extractServiceInfo(event);
    if (eventName == "ros2:rcl_client_init"sv)          return extractClientInfo(event);

    std::cout << "found known message type " << eventName << std::endl;

    return sharedMem::TraceMessage(sharedMem::MessageType::NONE);
}