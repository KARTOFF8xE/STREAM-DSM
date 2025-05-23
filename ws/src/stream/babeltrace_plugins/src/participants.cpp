#include "participants.hpp"

#include <ipc/util.hpp>
#include <ipc/sharedMem.hpp>
#include <sink.hpp>

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

sharedMem::TraceMessage extractTimerInitInfo(bt_self_component_sink *self_component_sink, const bt_event *event) {
    struct publisher *publisher = (struct publisher *)bt_self_component_get_data(
        bt_self_component_sink_as_self_component(self_component_sink)
    );

    const bt_field *payload_field = bt_event_borrow_payload_field_const(event);
    const bt_field_class *field_class = bt_field_borrow_class_const(payload_field);
    if (bt_field_class_get_type(field_class) == BT_FIELD_CLASS_TYPE_STRUCTURE) {
        const bt_field *keyField = bt_field_structure_borrow_member_field_by_name_const(payload_field, "timer_handle");
        const bt_field *valueField = bt_field_structure_borrow_member_field_by_name_const(payload_field, "period");
        publisher->timer_init[bt_field_integer_unsigned_get_value(keyField)]
            = bt_field_integer_signed_get_value(valueField);
    } else { printf("\033[33;1WRONG TYPE\033[0m\n"); }

    return sharedMem::TraceMessage(sharedMem::MessageType::NONE);
}

sharedMem::TraceMessage extractTimerLinkInfo(bt_self_component_sink *self_component_sink, const bt_event *event) {
    struct publisher *publisher = (struct publisher *)bt_self_component_get_data(
        bt_self_component_sink_as_self_component(self_component_sink)
    );
    sharedMem::TraceMessage msg = (sharedMem::MessageType::TIMERTRACE);

    const bt_field *payload_field = bt_event_borrow_payload_field_const(event);
    const bt_field_class *field_class = bt_field_borrow_class_const(payload_field);
    if (bt_field_class_get_type(field_class) == BT_FIELD_CLASS_TYPE_STRUCTURE) {
        const bt_field *field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "node_handle");
        msg.timer.nodeHandle = bt_field_integer_unsigned_get_value(field);
        field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "timer_handle");
        msg.timer.frequency = publisher->timer_init[bt_field_integer_unsigned_get_value(field)];
        publisher->timer_init.erase(bt_field_integer_unsigned_get_value(field));
    } else { printf("\033[33;1WRONG TYPE\033[0m\n"); }
    return msg;
}

sharedMem::TraceMessage extractLifecycleSMInitInfo(const bt_event *event) {
    sharedMem::TraceMessage msg = (sharedMem::MessageType::STATEMACHINEINITTRACE);

    const bt_field *payload_field = bt_event_borrow_payload_field_const(event);
    const bt_field_class *field_class = bt_field_borrow_class_const(payload_field);
    if (bt_field_class_get_type(field_class) == BT_FIELD_CLASS_TYPE_STRUCTURE) {
        const bt_field *field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "node_handle");
        msg.smInit.nodeHandle = bt_field_integer_unsigned_get_value(field);
        field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "state_machine");
        msg.smInit.stateMachine =  bt_field_integer_unsigned_get_value(field);
    } else { printf("\033[33;1WRONG TYPE\033[0m\n"); }
    return msg;
}

sharedMem::TraceMessage extractLifecycleTransitionInfo(const bt_event *event) {
    sharedMem::TraceMessage msg = (sharedMem::MessageType::STATETRANSITIONTRACE);

    const bt_field *payload_field = bt_event_borrow_payload_field_const(event);
    const bt_field_class *field_class = bt_field_borrow_class_const(payload_field);
    if (bt_field_class_get_type(field_class) == BT_FIELD_CLASS_TYPE_STRUCTURE) {
        const bt_field *field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "state_machine");
        msg.lcTrans.stateMachine = bt_field_integer_unsigned_get_value(field);
        field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "state_machine");
        field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "goal_label");
        std::string_view state = bt_field_string_get_value(field);

        using namespace std::string_view_literals;
        if (state == "unconfigured"sv)  { msg.lcTrans.state = sharedMem::LifeCycleState::UNCONFIGURED; return msg; }
        if (state == "inactive"sv)      { msg.lcTrans.state = sharedMem::LifeCycleState::INACTIVE;     return msg; }
        if (state == "active"sv)        { msg.lcTrans.state = sharedMem::LifeCycleState::ACTIVE;       return msg; }
        if (state == "finalized"sv)     { msg.lcTrans.state = sharedMem::LifeCycleState::FINALIZED;    return msg; }

    } else { printf("\033[33;1WRONG TYPE\033[0m\n"); }

    msg.lcTrans.state = sharedMem::LifeCycleState::INVALID;
    return msg;
}

void get_value(
    const bt_field *structure_field,
    const bt_field_class_structure_member *member
    ) {
    const bt_field_class *field_class = bt_field_class_structure_member_borrow_field_class_const(member);
    const char *name = bt_field_class_structure_member_get_name(member);
    const bt_field *field = bt_field_structure_borrow_member_field_by_name_const(structure_field, name);

    switch(bt_field_class_get_type(field_class)) {
        case BT_FIELD_CLASS_TYPE_STRING: {
            printf("%s (string)\n", bt_field_string_get_value(field));
            break;
        }
        case BT_FIELD_CLASS_TYPE_SIGNED_INTEGER: {
            printf("%ld (Int)\n", bt_field_integer_signed_get_value(field));
            break;
        }
        case BT_FIELD_CLASS_TYPE_UNSIGNED_INTEGER: {
            printf("%ld (UInt)\n", bt_field_integer_unsigned_get_value(field));
            break;
        }
        default: printf("\033[33;1UNKNOWN TYPE\033[0m\n");
    }
}

// Show every Element of a structure
void analyzeField(const bt_field *field) {
    const bt_field_class *field_class = bt_field_borrow_class_const(field);
    if (bt_field_class_get_type(field_class) == BT_FIELD_CLASS_TYPE_STRUCTURE) {
        uint64_t field_count = bt_field_class_structure_get_member_count(field_class);

        for (uint64_t i = 0; i < field_count; i++) {
            const bt_field_class_structure_member *member = bt_field_class_structure_borrow_member_by_index_const(field_class, i);
            printf("\t%s:\t", bt_field_class_structure_member_get_name(member));
            get_value(field, member);
        }

    } else { printf("\033[33;1UNKNOWN TYPE\033[0m\n"); }
}


sharedMem::TraceMessage extractTracedMessage(
    bt_self_component_sink *self_component_sink,
    const bt_message *message) {
    const bt_event *event = bt_message_event_borrow_event_const(message);
    const bt_event_class *eventClass = bt_event_borrow_class_const(event);

    std::string_view eventName(bt_event_class_get_name(eventClass));

    using namespace std::string_view_literals;
    if (eventName == "ros2:rcl_node_init"sv)            return extractNodeInfo(event);
    if (eventName == "ros2:rcl_publisher_init"sv)       return extractPublisherInfo(event);
    if (eventName == "ros2:rcl_subscription_init"sv)    return extractSubscriberInfo(event);
    if (eventName == "ros2:rcl_service_init"sv)         return extractServiceInfo(event);
    if (eventName == "ros2:rcl_client_init"sv)          return extractClientInfo(event);

    if (eventName == "ros2:rcl_timer_init"sv)           return extractTimerInitInfo(self_component_sink, event);
    if (eventName == "ros2:rclcpp_timer_link_node"sv)   return extractTimerLinkInfo(self_component_sink, event);

    if (eventName == "ros2:rcl_lifecycle_state_machine_init"sv) return extractLifecycleSMInitInfo(event);
    if (eventName == "ros2:rcl_lifecycle_transition"sv) return extractLifecycleTransitionInfo(event);


    /***unknown type***/
    std::cout << "found unknown message type " << eventName << std::endl;
        std::cout << bt_event_class_get_name(eventClass) << std::endl;
        /***analyze context***/
        const bt_field *ctx_field = bt_event_borrow_common_context_field_const(event);
        analyzeField(ctx_field);
        /***analyze payload***/
        const bt_field *payload_field = bt_event_borrow_payload_field_const(event);
        analyzeField(payload_field);


    return sharedMem::TraceMessage(sharedMem::MessageType::NONE);
}