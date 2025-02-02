#include <babeltrace2/babeltrace.h>

#include "interface.h"

struct publisher {
    bt_message_iterator *message_iterator;
    const bt_value *publisher_params;
};
 
/***Initializes Component***/
static bt_component_class_initialize_method_status publisher_initialize(
        bt_self_component_sink *self_component_sink,
        bt_self_component_sink_configuration *,
        const bt_value *params, void *);

/***Finalize Component (on destroy)***/
static void publisher_finalize(bt_self_component_sink *self_component_sink);

/***Configure Component***/
static bt_component_class_sink_graph_is_configured_method_status
publisher_graph_is_configured(bt_self_component_sink *self_component_sink);

/***extracting the value of a member***/
void get_value(
    const bt_field *structure_field,
    const bt_field_class_structure_member *member
    );

/***Show every Element of a structure***/
void analyzeField(const bt_field *field);

/***Publish the message (till now to the cli, later to another module of the tool)***/
static void publish(/*bt_self_component_sink *self_component_sink,*/ const bt_message *message);

/***Consumes the messages***/
bt_component_class_sink_consume_method_status publisher_consume(
        bt_self_component_sink *self_component_sink);