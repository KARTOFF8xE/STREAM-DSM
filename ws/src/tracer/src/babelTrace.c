#include <babeltrace2/babeltrace.h>
#include <stdio.h>
#include<unistd.h>

int load_Plugin(const char *plugin_name, const bt_plugin **plugin) {
    bt_plugin_find_status plugin_find_status = bt_plugin_find(
        plugin_name,
        BT_FALSE,
        BT_FALSE,
        BT_TRUE,
        BT_TRUE,
        BT_TRUE,
        plugin);
    switch (plugin_find_status) {
        case BT_PLUGIN_FIND_STATUS_OK: printf("Success"); break;
        case BT_PLUGIN_FIND_STATUS_NOT_FOUND: fprintf(stderr, "Plugin not found"); break;
        case BT_PLUGIN_FIND_STATUS_MEMORY_ERROR: fprintf(stderr, "Out of memory"); break;
        case BT_PLUGIN_FIND_STATUS_ERROR: fprintf(stderr, "Error"); break;
        default: printf("Hopefully never reached");
    }

    return 0;
}

int find_existing_instances_at_ip(const char *ip, bt_component_class_source *source_class_lttnglive, bt_value **result_map) {
    printf("\nCreate relayd_url-string.....");
    bt_value *relayd_url = bt_value_string_create();
    bt_value_string_set_status string_set_status = bt_value_string_set(relayd_url, ip);
    switch (string_set_status) {
        case BT_VALUE_STRING_SET_STATUS_OK: printf("Success"); break;
        case BT_VALUE_STRING_SET_STATUS_MEMORY_ERROR: fprintf(stderr, "Out of memory"); break;
        default: fprintf(stderr, "Hopefully never reached");
    }

    printf("\nInsert into param_map.....");
    bt_value *param_map = bt_value_map_create();
    bt_value_map_insert_entry_status map_insert_entry_status = bt_value_map_insert_entry(param_map, "url", relayd_url);
    switch (map_insert_entry_status) {
        case BT_VALUE_MAP_INSERT_ENTRY_STATUS_OK: printf("Success"); break;
        case BT_VALUE_MAP_INSERT_ENTRY_STATUS_MEMORY_ERROR: fprintf(stderr, "Out of memory"); break;
        default: fprintf(stderr, "Hopefully never reached");
    }

    printf("\nCreate Query Executor");
    bt_component_class *class_source_as_component_class = bt_component_class_source_as_component_class(source_class_lttnglive);
    bt_query_executor *query_executor = bt_query_executor_create(
        class_source_as_component_class,
        "sessions",
        param_map
    );
    if (!query_executor) {
        fprintf(stderr, ".....error");
    }

    printf("\nQuery Query-Executor");
    bt_value *result_array = bt_value_array_create();
    do {
        bt_query_executor_query_status query_executor_query_status = bt_query_executor_query(query_executor, (const bt_value**)&result_array);
        switch (query_executor_query_status) {
            case BT_QUERY_EXECUTOR_QUERY_STATUS_OK: printf("."); fflush(stdout); sleep(1); break;
            case BT_QUERY_EXECUTOR_QUERY_STATUS_UNKNOWN_OBJECT: printf(".....Unknown object to query"); break;
            case BT_QUERY_EXECUTOR_QUERY_STATUS_AGAIN: fprintf(stderr, ".....Try Again");  break;
            case BT_QUERY_EXECUTOR_QUERY_STATUS_MEMORY_ERROR: fprintf(stderr, ".....Out of memory"); break;
            case BT_QUERY_EXECUTOR_QUERY_STATUS_ERROR: fprintf(stderr, ".....Other Error"); break;
            default: fprintf(stderr, ".....Hopefully never reached");
        }
    } while (bt_value_array_is_empty(result_array) == BT_TRUE);
    printf("\nFound %ld client(s):", bt_value_array_get_length(result_array));

    *result_map = bt_value_array_borrow_element_by_index(result_array, 0);

    return 0;
}

int main() {
    /***Load Plugin source.ctl.lttng-live***/
    printf("Load Plugin ctl.lttng-live.....");
    const bt_plugin *plugin_lttnglive;
    load_Plugin("ctf", &plugin_lttnglive);

    const bt_component_class_source *source_class_lttnglive;
    source_class_lttnglive = bt_plugin_borrow_source_component_class_by_name_const(plugin_lttnglive, "lttng-live");

    /***Find existing instances at localhost***/
    bt_value *result_map;
    find_existing_instances_at_ip("net://localhost", (bt_component_class_source *)source_class_lttnglive, &result_map);
    bt_value *url_val = bt_value_map_borrow_entry_value(result_map, "url");
    if (!bt_value_is_string(url_val)) {
        fprintf(stderr, ".....Wrong type for param 'url' in Object.\n");
    }
    printf("\n\t%s", bt_value_string_get(url_val));

    /***Continue instantiating source.ctl.lttng-plugin***/
    printf("\nCreate Array value.....");
    bt_value *array_value = bt_value_array_create();
    bt_value_array_append_element_status array_append_element_status = bt_value_array_append_string_element(
        array_value,
        bt_value_string_get(url_val)
    );
    switch (array_append_element_status) {
        case BT_VALUE_ARRAY_APPEND_ELEMENT_STATUS_OK: printf("Success"); break;
        case BT_VALUE_ARRAY_APPEND_ELEMENT_STATUS_MEMORY_ERROR: fprintf(stderr, "Out of Memory"); break;
        default: fprintf(stderr, "Hopefully never reached");
    }

    printf("\nCreate Map value.....");
    bt_value *map_value = bt_value_map_create();
    bt_value_map_insert_entry_status map_insert_entry_status = bt_value_map_insert_entry(map_value, "inputs", array_value);
    switch (map_insert_entry_status) {
        case BT_VALUE_MAP_INSERT_ENTRY_STATUS_OK: printf("Success"); break;
        case BT_VALUE_MAP_INSERT_ENTRY_STATUS_MEMORY_ERROR: fprintf(stderr, "Out of Memory"); break;
        default: fprintf(stderr, "Hopefully never reached");
    }

    /***Load Plugin sink.text.details***/
    printf("\nLoad Plugin text.details.....");
    const bt_plugin *plugin_text;
    load_Plugin("text", &plugin_text);

    /***Create Graph and add Components***/
    printf("\nCreate Graph");
    bt_graph *graph = bt_graph_create(0);
    if (!graph) {
        fprintf(stderr, ".....Error: not able to create Graph");
        return 1;
    }

    printf("\nAdd source Component to Graph.....");
    const bt_component_source *source_lttnglive;
    bt_graph_add_component_status add_component_status = bt_graph_add_source_component(
        graph,
        source_class_lttnglive,
        "lttng-live",
        map_value,
        BT_LOGGING_LEVEL_WARNING,
        &source_lttnglive
    );
    switch (add_component_status) {
        case BT_GRAPH_ADD_COMPONENT_STATUS_OK: printf("Success"); break;
        case BT_GRAPH_ADD_COMPONENT_STATUS_MEMORY_ERROR: fprintf(stderr, "Out of Memory"); break;
        case BT_GRAPH_ADD_COMPONENT_STATUS_ERROR: fprintf(stderr, "Other error"); break;
        default: fprintf(stderr, "Hopefully never reached");
    }

    printf("\nAdd sink Component to Graph.....");
    const bt_component_class_sink *sink_class_details;
    sink_class_details = bt_plugin_borrow_sink_component_class_by_name_const(plugin_text, "pretty");

    const bt_component_sink *sink_details;
    add_component_status = bt_graph_add_sink_component(
        graph,
        sink_class_details,
        "details",
        NULL,
        BT_LOGGING_LEVEL_WARNING,
        &sink_details
    );
    switch (add_component_status) {
        case BT_GRAPH_ADD_COMPONENT_STATUS_OK: printf("Success"); break;
        case BT_GRAPH_ADD_COMPONENT_STATUS_MEMORY_ERROR: fprintf(stderr, "Out of Memory"); break;
        case BT_GRAPH_ADD_COMPONENT_STATUS_ERROR: fprintf(stderr, "Other error"); break;
        default: fprintf(stderr, "Hopefully never reached");
    }

    /***Connect Components***/
    printf("\nGet output port lttng-live/out");
    const bt_port_output *source_lttnglive_port_out;
    source_lttnglive_port_out = bt_component_source_borrow_output_port_by_name_const(source_lttnglive, "out");

    printf("\nGet input port details/in");
    const bt_port_input *sink_details_port_in;
    sink_details_port_in = bt_component_sink_borrow_input_port_by_name_const(sink_details, "in");

    printf("\nConnect ports lttng-live/out -> details/in.....");
    const bt_connection *connectionlttngdetails;
    bt_graph_connect_ports_status connect_ports_status = bt_graph_connect_ports(graph, source_lttnglive_port_out, sink_details_port_in, &connectionlttngdetails);
    switch (connect_ports_status) {
        case BT_GRAPH_CONNECT_PORTS_STATUS_OK: printf("Success"); break;
        case BT_GRAPH_CONNECT_PORTS_STATUS_MEMORY_ERROR: fprintf(stderr, "Out of memory"); break;
        case BT_GRAPH_CONNECT_PORTS_STATUS_ERROR: fprintf(stderr, "Other Error"); break;
        default: fprintf(stderr, "Hopefully never reached");
    }

    /***Run Graph**/
    printf("\nRun Graph\n");
    bt_graph_run_status graph_status;
    do {
        graph_status = bt_graph_run(graph);
    } while (graph_status == BT_GRAPH_RUN_STATUS_AGAIN);
    switch (connect_ports_status) {
        case BT_GRAPH_RUN_STATUS_OK: printf(".....Success"); break;
        case BT_GRAPH_RUN_STATUS_MEMORY_ERROR: fprintf(stderr, ".....Out of memory"); break;
        case BT_GRAPH_RUN_STATUS_ERROR: fprintf(stderr, ".....Other Error"); break;
        default: fprintf(stderr, ".....Hopefully never reached");
    }
}