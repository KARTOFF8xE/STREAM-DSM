#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <babeltrace2/babeltrace.h>

#include "interface.h"
#include "sink.h"
#include "participantFactory.h"

static bt_component_class_initialize_method_status publisher_initialize(
        bt_self_component_sink *self_component_sink,
        bt_self_component_sink_configuration *,
        const bt_value *params, void *) {

    struct publisher *publisher = (struct publisher *)malloc(sizeof(*publisher));
 
    bt_self_component_set_data(
        bt_self_component_sink_as_self_component(self_component_sink),
        publisher);

    bt_self_component_sink_add_input_port(self_component_sink,
        "in", NULL, NULL);
 
    return BT_COMPONENT_CLASS_INITIALIZE_METHOD_STATUS_OK;
}

static void publisher_finalize(bt_self_component_sink *self_component_sink) {
    struct publisher *publisher = (struct publisher *)bt_self_component_get_data(
        bt_self_component_sink_as_self_component(self_component_sink));
 
    free(publisher);
}

static bt_component_class_sink_graph_is_configured_method_status
publisher_graph_is_configured(bt_self_component_sink *self_component_sink) {
    struct publisher *publisher = (struct publisher *)bt_self_component_get_data(
        bt_self_component_sink_as_self_component(self_component_sink));
 
    bt_self_component_port_input *in_port =
        bt_self_component_sink_borrow_input_port_by_index(
            self_component_sink, 0);
 
    bt_message_iterator_create_from_sink_component(self_component_sink,
        in_port, &publisher->message_iterator);
 
    return BT_COMPONENT_CLASS_SINK_GRAPH_IS_CONFIGURED_METHOD_STATUS_OK;
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
            printf("%ld (Uint)\n", bt_field_integer_signed_get_value(field));
            break;
        }
        case BT_FIELD_CLASS_TYPE_UNSIGNED_INTEGER: {
            printf("%ld (Int)\n", bt_field_integer_unsigned_get_value(field));
            break;
        }
        default: printf("\033[33;1UNKNOWN TYPE\033[0m\n");
    }
}

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

static void publish(/*bt_self_component_sink *self_component_sink,*/ const bt_message *message) {
    const bt_event *event = bt_message_event_borrow_event_const(message);
    const bt_event_class *event_class = bt_event_borrow_class_const(event);

    IParticipant *participant;
    participant = ParticipantFactory::getParticipant(bt_event_class_get_name(event_class));

    participant->extractInfo(event);
    participant->toGraph();

    return;

    // /***unknown Topic***/
    // std::cout << "unknown topic" << std::endl;
    //     std::cout << bt_event_class_get_name(event_class) << std::endl;
    //     /***analyze context***/
    //     const bt_field *ctx_field = bt_event_borrow_common_context_field_const(event);
    //     analyzeField(ctx_field);
    //     /***analyze payload***/
    //     const bt_field *payload_field = bt_event_borrow_payload_field_const(event);
    //     analyzeField(payload_field);
}

bt_component_class_sink_consume_method_status publisher_consume(
        bt_self_component_sink *self_component_sink) {
    struct publisher *publisher = (struct publisher *)bt_self_component_get_data(
        bt_self_component_sink_as_self_component(self_component_sink));
 
    bt_message_array_const messages;
    uint64_t message_count;
    bt_message_iterator_next_status next_status =
        bt_message_iterator_next(publisher->message_iterator, &messages,
            &message_count);
 
    switch (next_status) {
        case BT_MESSAGE_ITERATOR_NEXT_STATUS_END:
            bt_message_iterator_put_ref(publisher->message_iterator);
            return BT_COMPONENT_CLASS_SINK_CONSUME_METHOD_STATUS_END;
        case BT_MESSAGE_ITERATOR_NEXT_STATUS_AGAIN:
            return BT_COMPONENT_CLASS_SINK_CONSUME_METHOD_STATUS_AGAIN;
        case BT_MESSAGE_ITERATOR_NEXT_STATUS_MEMORY_ERROR:
            return BT_COMPONENT_CLASS_SINK_CONSUME_METHOD_STATUS_MEMORY_ERROR;
        case BT_MESSAGE_ITERATOR_NEXT_STATUS_ERROR:
            return BT_COMPONENT_CLASS_SINK_CONSUME_METHOD_STATUS_ERROR;
        default:
            break;
    }
 
    for (uint64_t i = 0; i < message_count; i++) {
        const bt_message *message = messages[i];

        if (bt_message_get_type(message) == BT_MESSAGE_TYPE_EVENT) {
            publish(/*self_component_sink,*/ message);
        }
 
        bt_message_put_ref(message);
    }

    return BT_COMPONENT_CLASS_SINK_CONSUME_METHOD_STATUS_OK;
}

BT_PLUGIN_MODULE();

BT_PLUGIN(publisher);

BT_PLUGIN_AUTHOR("KARTOFF8xE");
BT_PLUGIN_DESCRIPTION("This Plugin publishes a message onto a given Topic.");
BT_PLUGIN_VERSION(0, 0, 1, "dev");

BT_PLUGIN_SINK_COMPONENT_CLASS(output, publisher_consume);

BT_PLUGIN_SINK_COMPONENT_CLASS_INITIALIZE_METHOD(output,
    publisher_initialize);
BT_PLUGIN_SINK_COMPONENT_CLASS_FINALIZE_METHOD(output, publisher_finalize);
BT_PLUGIN_SINK_COMPONENT_CLASS_GRAPH_IS_CONFIGURED_METHOD(output,
    publisher_graph_is_configured);