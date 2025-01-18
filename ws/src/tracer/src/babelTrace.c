#include <babeltrace2/babeltrace.h>
#include <stdio.h>

int main() {
    printf("Load Plugin\n");
    const bt_plugin *plugin;
    bt_plugin_find_status plugin_find_status = bt_plugin_find(
        "ctf",
        BT_FALSE,
        BT_FALSE,
        BT_TRUE,
        BT_TRUE,
        BT_TRUE,
        &plugin);
    switch (plugin_find_status) {
        case BT_PLUGIN_FIND_STATUS_OK: printf("\tSuccess.\n"); break;
        case BT_PLUGIN_FIND_STATUS_NOT_FOUND: fprintf(stderr, "\tPlugin not found.\n"); break;
        case BT_PLUGIN_FIND_STATUS_MEMORY_ERROR: fprintf(stderr, "\tOut of memory.\n"); break;
        case BT_PLUGIN_FIND_STATUS_ERROR: fprintf(stderr, "\tError.\n"); break;
        default: printf("\tHopefully never reached\n");
    }

    const bt_component_class_source *component_class_source;
    component_class_source = bt_plugin_borrow_source_component_class_by_name_const(plugin, "lttng-live");

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

    printf("Add Component to Graph\n");
    const bt_component_source *component_source;
    bt_graph_add_component_status add_source_component_status = bt_graph_add_source_component(
        graph,
        component_class_source,
        "lttng-live-foo",
        map_value,
        BT_LOGGING_LEVEL_TRACE,
        &component_source
    );
    switch (add_source_component_status) {
        case BT_GRAPH_ADD_COMPONENT_STATUS_OK: printf("\tSuccess.\n"); break;
        case BT_GRAPH_ADD_COMPONENT_STATUS_MEMORY_ERROR: fprintf(stderr, "Out of Memory.\n"); break;
        case BT_GRAPH_ADD_COMPONENT_STATUS_ERROR: fprintf(stderr, "\tOther error.\n"); break;
        default: printf("\tHopefully never reached\n");
    }
    
    printf("Number of Ports the Source Component has: %ld\n", bt_component_source_get_output_port_count(component_source));
    //-------------------------------

    




    // printf("Run Graph\n");
    // bt_graph_run_status graph_status = bt_graph_run(graph);
    // if (graph_status != 0) {
    //     fprintf(stderr, "\tsome error accured on running graph");
    // }
}