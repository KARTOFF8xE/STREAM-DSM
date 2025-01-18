#include <babeltrace2/babeltrace.h>
#include <stdio.h>

int main() {
    printf("Load Plugin ctl.lttng-live\n");
    const bt_plugin *plugin_lttnglive;
    bt_plugin_find_status plugin_find_status = bt_plugin_find(
        "ctf",
        BT_FALSE,
        BT_FALSE,
        BT_TRUE,
        BT_TRUE,
        BT_TRUE,
        &plugin_lttnglive);
    switch (plugin_find_status) {
        case BT_PLUGIN_FIND_STATUS_OK: printf("\tSuccess.\n"); break;
        case BT_PLUGIN_FIND_STATUS_NOT_FOUND: fprintf(stderr, "\tPlugin not found.\n"); break;
        case BT_PLUGIN_FIND_STATUS_MEMORY_ERROR: fprintf(stderr, "\tOut of memory.\n"); break;
        case BT_PLUGIN_FIND_STATUS_ERROR: fprintf(stderr, "\tError.\n"); break;
        default: printf("\tHopefully never reached\n");
    }

    const bt_component_class_source *source_class_lttnglive;
    source_class_lttnglive = bt_plugin_borrow_source_component_class_by_name_const(plugin_lttnglive, "lttng-live");

    printf("Create Array value\n");
    bt_value *array_value = bt_value_array_create();
    bt_value_array_append_element_status array_append_element_status = bt_value_array_append_string_element(array_value, "net://127.0.0.1");
    switch (array_append_element_status) {
        case BT_VALUE_ARRAY_APPEND_ELEMENT_STATUS_OK: printf("\tSuccess.\n"); break;
        case BT_VALUE_ARRAY_APPEND_ELEMENT_STATUS_MEMORY_ERROR: fprintf(stderr, "\tOut of Memory.\n"); break;
        default: printf("\tHopefully never reached\n");
    }
    // printf("Length of array: %ld\n", bt_value_array_get_length(array_value));

    printf("Create Map value\n");
    bt_value *map_value = bt_value_map_create();
    bt_value_map_insert_entry_status map_insert_entry_status = bt_value_map_insert_entry(map_value, "inputs", array_value);
    switch (map_insert_entry_status) {
        case BT_VALUE_MAP_INSERT_ENTRY_STATUS_OK: printf("\tSuccess.\n"); break;
        case BT_VALUE_MAP_INSERT_ENTRY_STATUS_MEMORY_ERROR: fprintf(stderr, "\tOut of Memory.\n"); break;
        default: printf("\tHopefully never reached\n");
    }
    // printf("Map has Entry 'inputs': %d\tSize of Map: %ld\n", bt_value_map_has_entry(map_value, "inputs"), bt_value_map_get_size(map_value));

    printf("Create Graph\n");
    bt_graph *graph = bt_graph_create(0);
    if (!graph) {
        fprintf(stderr, "Fehler: Graph konnte nicht erstellt werden.\n");
        return 1;
    }

    printf("Add source Component to Graph\n");
    const bt_component_source *source_lttnglive;
    bt_graph_add_component_status add_component_status = bt_graph_add_source_component(
        graph,
        source_class_lttnglive,
        "lttng-live-foo",
        map_value,
        BT_LOGGING_LEVEL_TRACE,
        &source_lttnglive
    );
    switch (add_component_status) {
        case BT_GRAPH_ADD_COMPONENT_STATUS_OK: printf("\tSuccess.\n"); break;
        case BT_GRAPH_ADD_COMPONENT_STATUS_MEMORY_ERROR: fprintf(stderr, "Out of Memory.\n"); break;
        case BT_GRAPH_ADD_COMPONENT_STATUS_ERROR: fprintf(stderr, "\tOther error.\n"); break;
        default: printf("\tHopefully never reached\n");
    }
    
    printf("Number of Ports the Source Component has: %ld\n", bt_component_source_get_output_port_count(source_lttnglive));
    //-------------------------------

    
    printf("Load Plugin text.details\n");
    const bt_plugin *plugin_text;
    plugin_find_status = bt_plugin_find(
        "text",
        BT_FALSE,
        BT_FALSE,
        BT_TRUE,
        BT_TRUE,
        BT_TRUE,
        &plugin_text);
    switch (plugin_find_status) {
        case BT_PLUGIN_FIND_STATUS_OK: printf("\tSuccess.\n"); break;
        case BT_PLUGIN_FIND_STATUS_NOT_FOUND: fprintf(stderr, "\tPlugin not found.\n"); break;
        case BT_PLUGIN_FIND_STATUS_MEMORY_ERROR: fprintf(stderr, "\tOut of memory.\n"); break;
        case BT_PLUGIN_FIND_STATUS_ERROR: fprintf(stderr, "\tError.\n"); break;
        default: printf("\tHopefully never reached\n");
    }

    const bt_component_class_sink *sink_class_details;
    sink_class_details = bt_plugin_borrow_sink_component_class_by_name_const(plugin_text, "details");

    printf("Add sink Component to Graph\n");
    const bt_component_sink *sink_details;
    add_component_status = bt_graph_add_sink_component(
        graph,
        sink_class_details,
        "details-foo",
        NULL,
        BT_LOGGING_LEVEL_TRACE,
        &sink_details
    );
    switch (add_component_status) {
        case BT_GRAPH_ADD_COMPONENT_STATUS_OK: printf("\tSuccess.\n"); break;
        case BT_GRAPH_ADD_COMPONENT_STATUS_MEMORY_ERROR: fprintf(stderr, "Out of Memory.\n"); break;
        case BT_GRAPH_ADD_COMPONENT_STATUS_ERROR: fprintf(stderr, "\tOther error.\n"); break;
        default: printf("\tHopefully never reached\n");
    }

    printf("Number of Ports the Sink Component has: %ld\n", bt_component_sink_get_input_port_count(sink_details));



    // printf("Run Graph\n");
    // bt_graph_run_status graph_status = bt_graph_run(graph);
    // if (graph_status != 0) {
    //     fprintf(stderr, "\tsome error accured on running graph");
    // }
}