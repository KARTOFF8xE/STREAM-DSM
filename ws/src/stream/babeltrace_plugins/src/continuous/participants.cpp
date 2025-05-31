#include "continuous/participants.hpp"

#include <ipc/util.hpp>
#include <ipc/sharedMem.hpp>
#include <continuous/sink.hpp>
#include <vector>
#include <influxdb/influxdb.hpp>
#include <curl/myCurl.hpp>
#include <neo4j/publisher/publisher.hpp>
#include <nlohmann/json.hpp>

#include <string_view>


sharedMem::TraceMessage extractPublishInfo(bt_self_component_sink *self_component_sink, const bt_event *event) {
    struct tracer *tracer = (struct tracer *)bt_self_component_get_data(
        bt_self_component_sink_as_self_component(self_component_sink)
    );

    const bt_field *payload_field = bt_event_borrow_payload_field_const(event);
    const bt_field_class *field_class = bt_field_borrow_class_const(payload_field);
    if (bt_field_class_get_type(field_class) == BT_FIELD_CLASS_TYPE_STRUCTURE) {
        const bt_field *publisherHandle = bt_field_structure_borrow_member_field_by_name_const(payload_field, "publisher_handle");
        tracer->publishingRate[bt_field_integer_unsigned_get_value(publisherHandle)]++;
    } else { printf("\033[33;1WRONG TYPE\033[0m\n"); }

    return sharedMem::TraceMessage(sharedMem::MessageType::NONE);
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

    if (eventName == "ros2:rcl_publish"sv)  return extractPublishInfo(self_component_sink, event);


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