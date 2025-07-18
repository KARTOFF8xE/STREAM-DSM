#include <babeltrace2/babeltrace.h>
#include <stdio.h>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <lttng/lttng.h>
#include <csignal>
#include <vector>
#include <sys/mman.h>


void handle_sigint(int) {
    shm_unlink("/babeltonato");
    lttng_destroy_session("structuralSession");
    exit(EXIT_SUCCESS);
}


int load_Plugin(const char *plugin_name, const bt_plugin **plugin) {
    bt_plugin_find_status plugin_find_status = bt_plugin_find(
        plugin_name,
        BT_FALSE,
        BT_TRUE,
        BT_TRUE,
        BT_TRUE,
        BT_TRUE,
        plugin);
    switch (plugin_find_status) {
        case BT_PLUGIN_FIND_STATUS_OK: printf("\033[32;1mSuccess\033[0m\n"); break;
        case BT_PLUGIN_FIND_STATUS_NOT_FOUND: fprintf(stderr, "\033[31;1mPlugin not found\033[0m\n"); break;
        case BT_PLUGIN_FIND_STATUS_MEMORY_ERROR: fprintf(stderr, "\033[31;1mOut of memory\033[0m\n"); break;
        case BT_PLUGIN_FIND_STATUS_ERROR: fprintf(stderr, "\033[31;1mError\033[0m\n"); break;
        default: printf("\033[31;1mHopefully never reached\033[0m\n");
    }

    return 0;
}

int find_existing_instances_at_ip(const char *ip, bt_component_class_source *source_class_lttnglive, bt_value **result_map) {
    printf("Create relayd_url-string....."); fflush(stdout);
    bt_value *relayd_url = bt_value_string_create();
    bt_value_string_set_status string_set_status = bt_value_string_set(relayd_url, ip);
    switch (string_set_status) {
        case BT_VALUE_STRING_SET_STATUS_OK: printf("\033[32;1mSuccess\033[0m\n"); break;
        case BT_VALUE_STRING_SET_STATUS_MEMORY_ERROR: fprintf(stderr, "\033[31;1mOut of memory\033[0m\n"); break;
        default: fprintf(stderr, "\033[31;1mHopefully never reached\033[0m\n");
    }

    printf("Insert into param_map....."); fflush(stdout);
    bt_value *param_map = bt_value_map_create();
    bt_value_map_insert_entry_status map_insert_entry_status = bt_value_map_insert_entry(param_map, "url", relayd_url);
    switch (map_insert_entry_status) {
        case BT_VALUE_MAP_INSERT_ENTRY_STATUS_OK: printf("\033[32;1mSuccess\033[0m\n"); break;
        case BT_VALUE_MAP_INSERT_ENTRY_STATUS_MEMORY_ERROR: fprintf(stderr, "\033[31;1mOut of memory\033[0m\n"); break;
        default: fprintf(stderr, "\033[31;1mHopefully never reached\033[0m\n");
    }

    printf("Create and Query Query-Executor"); fflush(stdout);
    bt_value *result_array = bt_value_array_create();
    do {
        bt_component_class *class_source_as_component_class = bt_component_class_source_as_component_class(source_class_lttnglive);
        bt_query_executor *query_executor = bt_query_executor_create(
            class_source_as_component_class,
            "sessions",
            param_map
        );
        if (!query_executor) {
            fprintf(stderr, ".....error");
        }

        bt_query_executor_query_status query_executor_query_status = bt_query_executor_query(query_executor, (const bt_value**)&result_array);
        switch (query_executor_query_status) {
            case BT_QUERY_EXECUTOR_QUERY_STATUS_OK: printf("."); fflush(stdout); sleep(1); break;
            case BT_QUERY_EXECUTOR_QUERY_STATUS_UNKNOWN_OBJECT: printf("\033[31;1m.....Unknown object to query\033[0m\n"); break;
            case BT_QUERY_EXECUTOR_QUERY_STATUS_AGAIN: fprintf(stderr, "\033[31;1m.....Try Again\033[0m\n");  break;
            case BT_QUERY_EXECUTOR_QUERY_STATUS_MEMORY_ERROR: fprintf(stderr, "\033[31;1m.....Out of memory\033[0m\n"); break;
            case BT_QUERY_EXECUTOR_QUERY_STATUS_ERROR: fprintf(stderr, "\033[31;1m.....Other Error (there might not be a lttng instance yet, start it first)\033[0m\n"); break;
            default: fprintf(stderr, "\033[31;1m.....Hopefully never reached\033[0m\n");
        }
    } while (bt_value_array_is_empty(result_array) == BT_TRUE);
    printf("\nFound %ld client(s):\n", bt_value_array_get_length(result_array));

    *result_map = bt_value_array_borrow_element_by_index(result_array, 0);

    return 0;
}

std::vector<lttng_event*> enable_events(lttng_handle *handle, const char *channel_name) {
    const char *event_names[] = {
        "ros2:rcl_node_init",
        "ros2:rcl_publisher_init",
        "ros2:rcl_subscription_init",
        "ros2:rcl_service_init",
        "ros2:rcl_client_init",
        "ros2:rcl_timer_init",
        "ros2:rclcpp_timer_link_node",
        "ros2:rcl_lifecycle_state_machine_init",
        "ros2:rcl_lifecycle_transition",
        // "ros2:rclcpp_subscription_callback_added",
        // "ros2:rclcpp_subscription_init",
        // "ros2:rclcpp_service_callback_added",
        // "ros2:rclcpp_timer_callback_added",
        // "ros2:callback_start",
        // "ros2:callback_end",
    };

    std::vector<lttng_event*> enabled_events;

    for (const char *event_name : event_names) {
        lttng_event *event = lttng_event_create();
        if (!event) {
            std::cerr << "Failed to create LTTng event for " << event_name << std::endl;
            break;
        }

        std::strncpy(event->name, event_name, LTTNG_SYMBOL_NAME_LEN);
        int ret = lttng_enable_event(handle, event, channel_name);
        if (ret != 0) {
            std::cerr << "Failed to enable event '" << event_name << "': " << lttng_strerror(ret) << std::endl;
            lttng_event_destroy(event);
            break;
        }

        enabled_events.push_back(event);
    }

    return enabled_events;
}


int main() {
    signal(SIGINT, handle_sigint);
    lttng_destroy_session("structuralSession");

    int ret = lttng_create_session_live("structuralSession", "net://localhost", 1000000);
    if (ret < 0) {
        std::cerr << "Failed to create LTTng session: " << lttng_strerror(ret) << std::endl;
        return 1;
    }
    std::cout << "LTTng session created successfully." << std::endl;

    lttng_domain lttngDomain {
        .type       = lttng_domain_type::LTTNG_DOMAIN_UST,
        .buf_type   = lttng_buffer_type::LTTNG_BUFFER_PER_UID
    };
    lttng_handle *lttngHandle = lttng_create_handle("structuralSession", &lttngDomain);

    lttng_channel *lttngChannel = lttng_channel_create(&lttngDomain);
    lttngChannel->attr.subbuf_size = 524288;
    strncpy(lttngChannel->name, "structuralChannel", LTTNG_SYMBOL_NAME_LEN);
    ret = lttng_enable_channel(lttngHandle, lttngChannel);
    if (ret < 0) {
        std::cerr << "Failed to enable LTTng channel: " << lttng_strerror(ret) << std::endl;
        return 1;
    }

    std::vector<lttng_event *> events = enable_events(lttngHandle, lttngChannel->name);

    lttng_event_context ctx;
    ctx.ctx = lttng_event_context_type::LTTNG_EVENT_CONTEXT_PROCNAME;
    lttng_add_context(lttngHandle, &ctx, "ros2:rcl_node_init", "structuralChannel");
    lttng_event_context ctx2;
    ctx2.ctx = lttng_event_context_type::LTTNG_EVENT_CONTEXT_VPID;
    lttng_add_context(lttngHandle, &ctx2, "ros2:rcl_node_init", "structuralChannel");

    ret = lttng_start_tracing("structuralSession");
    if (ret != 0) {
        std::cerr << "Failed to start tracing: " << lttng_strerror(ret) << std::endl;
        return 1;
    }

    // set log level for debugging:
    // bt_logging_set_global_level(bt_logging_level::BT_LOGGING_LEVEL_DEBUG);

    /***Load Plugin source.ctf.lttng-live***/
    printf("Load Plugin ctf.lttng-live....."); fflush(stdout);
    const bt_plugin *plugin_lttnglive;
    load_Plugin("ctf", &plugin_lttnglive);

    const bt_component_class_source *source_class_lttnglive;
    source_class_lttnglive = bt_plugin_borrow_source_component_class_by_name_const(plugin_lttnglive, "lttng-live");

    /***Find existing instances at localhost***/
    bt_value *result_map;
    find_existing_instances_at_ip("net://localhost", (bt_component_class_source *)source_class_lttnglive, &result_map);
    bt_value *url_val = bt_value_map_borrow_entry_value(result_map, "url");
    if (!bt_value_is_string(url_val)) {
        fprintf(stderr, "\033[31;1m.....Wrong type for param 'url' in Object.\033[0m\n");
    }
    printf("\t\033[33m%s\033[0m\n", bt_value_string_get(url_val));

    /***Load Plugin sink.structural.details***/
    printf("Load Plugin structural.details....."); fflush(stdout);
    const bt_plugin *plugin_text;
    load_Plugin("structural", &plugin_text);

    /***Create Graph and add Components***/
    printf("Create Graph\n"); fflush(stdout);
    bt_graph *graph = bt_graph_create(0);
    if (!graph) {
        fprintf(stderr, "\033[31;1m.....Error: not able to create Graph\033[0m\n");
        return 1;
    }

    printf("Add source Component to Graph....."); fflush(stdout);
    const bt_component_source *source_lttnglive;
    {
        bt_value *array_value = bt_value_array_create();
        bt_value_array_append_element_status array_append_element_status = bt_value_array_append_string_element(
            array_value,
            bt_value_string_get(url_val)
        );
        switch (array_append_element_status) {
            case BT_VALUE_ARRAY_APPEND_ELEMENT_STATUS_OK: break;
            case BT_VALUE_ARRAY_APPEND_ELEMENT_STATUS_MEMORY_ERROR: fprintf(stderr, "\033[31;1mOut of Memory at at configuration\033[0m\n"); break;
            default: fprintf(stderr, "\033[31;1mHopefully never reached\033[0m\n");
        }
        bt_value *map_value = bt_value_map_create();
        bt_value_map_insert_entry_status map_insert_entry_status = bt_value_map_insert_entry(map_value, "inputs", array_value);
        switch (map_insert_entry_status) {
            case BT_VALUE_MAP_INSERT_ENTRY_STATUS_OK: break;
            case BT_VALUE_MAP_INSERT_ENTRY_STATUS_MEMORY_ERROR: fprintf(stderr, "\033[31;1mOut of Memory at at configuration\033[0m\n"); break;
            default: fprintf(stderr, "\033[31;1mHopefully never reached\033[0m\n");
        }
        bt_graph_add_component_status add_component_status = bt_graph_add_source_component(
            graph,
            source_class_lttnglive,
            "lttng-live",
            map_value,
            BT_LOGGING_LEVEL_WARNING,
            &source_lttnglive
        );
        switch (add_component_status) {
            case BT_GRAPH_ADD_COMPONENT_STATUS_OK: printf("\033[32;1mSuccess\033[0m\n"); break;
            case BT_GRAPH_ADD_COMPONENT_STATUS_MEMORY_ERROR: fprintf(stderr, "\033[31;1mOut of Memory\033[0m\n"); break;
            case BT_GRAPH_ADD_COMPONENT_STATUS_ERROR: fprintf(stderr, "\033[31;1mOther error\033[0m\n"); break;
            default: fprintf(stderr, "\033[31;1mHopefully never reached\033[0m\n");
        }
    }

    printf("Add sink Component to Graph....."); fflush(stdout);
    const bt_component_sink *sink_details;
    {
        bt_value *map_value = bt_value_map_create();

        const bt_component_class_sink *sink_class_details;
        sink_class_details = bt_plugin_borrow_sink_component_class_by_name_const(plugin_text, "output");

        bt_graph_add_component_status add_component_status = bt_graph_add_sink_component(
            graph,
            sink_class_details,
            "details",
            map_value,
            BT_LOGGING_LEVEL_WARNING,
            &sink_details
        );
        switch (add_component_status) {
            case BT_GRAPH_ADD_COMPONENT_STATUS_OK: printf("\033[32;1mSuccess\033[0m\n"); break;
            case BT_GRAPH_ADD_COMPONENT_STATUS_MEMORY_ERROR: fprintf(stderr, "\033[31;1mOut of Memory\033[0m\n"); break;
            case BT_GRAPH_ADD_COMPONENT_STATUS_ERROR: fprintf(stderr, "\033[31;1mOther error\033[0m\n"); break;
            default: fprintf(stderr, "\033[31;1mHopefully never reached\033[0m\n");
        }
    }

    /***Connect Components***/
    printf("Get output port lttng-live/out\n");
    const bt_port_output *source_lttnglive_port_out;
    source_lttnglive_port_out = bt_component_source_borrow_output_port_by_name_const(source_lttnglive, "out");

    printf("Get input port structural/in\n");
    const bt_port_input *sink_details_port_in;
    sink_details_port_in = bt_component_sink_borrow_input_port_by_name_const(sink_details, "in");

    printf("Connect ports lttng-live/out -> structural/in....."); fflush(stdout);
    const bt_connection *connectionlttngdetails;
    bt_graph_connect_ports_status connect_ports_status = bt_graph_connect_ports(graph, source_lttnglive_port_out, sink_details_port_in, &connectionlttngdetails);
    switch (connect_ports_status) {
        case BT_GRAPH_CONNECT_PORTS_STATUS_OK: printf("\033[32;1mSuccess\033[0m\n"); break;
        case BT_GRAPH_CONNECT_PORTS_STATUS_MEMORY_ERROR: fprintf(stderr, "\033[31;1mOut of memory\033[0m\n"); break;
        case BT_GRAPH_CONNECT_PORTS_STATUS_ERROR: fprintf(stderr, "\033[31;1mOther Error\033[0m\n"); break;
        default: fprintf(stderr, "\033[31;1mHopefully never reached\033[0m\n");
    }

    /***Run Graph**/
    printf("Run Graph\n");
    bt_graph_run_status graph_status;
    do {
        graph_status = bt_graph_run(graph);
    } while (graph_status == BT_GRAPH_RUN_STATUS_AGAIN);
    switch (graph_status) {
        case BT_GRAPH_RUN_STATUS_OK: printf("\033[32;1m.....Success\033[0m\n"); break;
        case BT_GRAPH_RUN_STATUS_MEMORY_ERROR: fprintf(stderr, "\033[31;1m.....Out of memory\033[0m\n"); break;
        case BT_GRAPH_RUN_STATUS_ERROR: fprintf(stderr, "\033[31;1m.....Other Error\033[0m\n"); break;
        default: fprintf(stderr, "\033[31;1m.....Hopefully never reached\033[0m\n");
    }
}