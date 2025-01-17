#include <babeltrace2/babeltrace.h>
#include <stdio.h>

int main() {
    const bt_plugin *plugin;
    bt_plugin_find_status status = bt_plugin_find(
        "ctf",
        BT_FALSE,
        BT_FALSE,
        BT_TRUE,
        BT_TRUE,
        BT_TRUE,
        &plugin);

    if (status == BT_PLUGIN_FIND_STATUS_OK) {
        printf("Success.\n");
    }
    if (status == BT_PLUGIN_FIND_STATUS_NOT_FOUND) {
        printf("Plugin not found.\n");
    }
    if (status == BT_PLUGIN_FIND_STATUS_MEMORY_ERROR) {
        printf("Out of memory.\n");
    }
    if (status == BT_PLUGIN_FIND_STATUS_ERROR) {
        printf("Error.\n");
    }

    bt_graph *graph = NULL;
    const char *url = "net://127.0.0.1"; // URL des Relay-Daemons
    bt_value *params = NULL;

    // Graph erstellen
    graph = bt_graph_create(0);
    if (!graph) {
        fprintf(stderr, "Fehler: Graph konnte nicht erstellt werden.\n");
        return 1;
    }

    if (bt_plugin_borrow_source_component_class_by_name_const(plugin, "lttng-live") != 0) {
        printf("Some Error at bt_plugin_borrow_source_component_class_by_name_const.\n");
    }
    
    // Parameter setzen (Relay-Daemon URL)
    params = bt_value_map_create();
    if (!params) {
        fprintf(stderr, "Fehler: Parameter-Map konnte nicht erstellt werden.\n");
        bt_plugin_put_ref(plugin);
        bt_graph_put_ref(graph);
        return 2;
    }

    bt_value_map_insert_string_entry(params, "url", url);
}