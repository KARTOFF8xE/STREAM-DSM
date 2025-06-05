#include "babeltrace.hpp"

#include <babeltrace2/babeltrace.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>


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
        case BT_PLUGIN_FIND_STATUS_OK: break; ("\033[32;1mSuccess\033[0m\n"); break;
        case BT_PLUGIN_FIND_STATUS_NOT_FOUND: fprintf(stderr, "\033[31;1mPlugin not found\033[0m\n"); break;
        case BT_PLUGIN_FIND_STATUS_MEMORY_ERROR: fprintf(stderr, "\033[31;1mOut of memory\033[0m\n"); break;
        case BT_PLUGIN_FIND_STATUS_ERROR: fprintf(stderr, "\033[31;1mError\033[0m\n"); break;
        default: printf("\033[31;1mHopefully never reached\033[0m\n");
    }

    return 0;
}

std::vector<std::string> find_trace_dirs_with_metadata(const std::string& root_path) {
    std::vector<std::string> result;

    try {
        for (const auto& entry : std::filesystem::recursive_directory_iterator(root_path)) {
            if (entry.is_directory()) {
                std::filesystem::path metadata_path = entry.path() / "metadata";
                if (std::filesystem::exists(metadata_path) && std::filesystem::is_regular_file(metadata_path)) {
                    result.push_back(std::filesystem::absolute(entry.path()).string());
                }
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Fehler beim Durchsuchen des Verzeichnisses: " << e.what() << std::endl;
    }

    return result;
}


int createAndExecuteTraceGraph(const char *pathToTraceDir) {
    std::vector<std::string> traces = find_trace_dirs_with_metadata(pathToTraceDir);

    // set log level for debugging:
    // bt_logging_set_global_level(bt_logging_level::BT_LOGGING_LEVEL_DEBUG);

    /***Load Plugin source.ctf.fs***/
    // printf("Load Plugin ctf.fs....."); fflush(stdout);
    const bt_plugin *ctlFs;
    load_Plugin("ctf", &ctlFs);

    const bt_component_class_source *source_class_lttnglive;
    source_class_lttnglive = bt_plugin_borrow_source_component_class_by_name_const(ctlFs, "fs");

    /***Load Plugin filter.utils.muxer***/
    // printf("Load Plugin utils.muxer....."); fflush(stdout);
    const bt_plugin *muxer;
    load_Plugin("utils", &muxer);

    const bt_component_class_filter *filter_class_muxer;
    filter_class_muxer = bt_plugin_borrow_filter_component_class_by_name_const(muxer, "muxer");

    /***Load Plugin sink.structural.details***/
    // printf("Load Plugin text.details....."); fflush(stdout);
    const bt_plugin *plugin_text;
    load_Plugin("continuous", &plugin_text);

    const bt_component_class_sink *sink_class_text;
    sink_class_text = bt_plugin_borrow_sink_component_class_by_name_const(plugin_text, "output");

    /***Create Graph and add Components***/
    // printf("Create Graph\n"); fflush(stdout);
    bt_graph *graph = bt_graph_create(0);
    if (!graph) {
        fprintf(stderr, "\033[31;1m.....Error: not able to create Graph\033[0m\n");
        return 1;
    }

    // printf("Add source Components to Graph....."); fflush(stdout);
    // const bt_component_source *source_lttnglive;
    std::vector<const bt_component_source *> sources_fs;
    for (size_t i = 0; i < traces.size(); i++) {
        bt_value *array_value = bt_value_array_create();

        bt_value *directory = bt_value_string_create();
        bt_value_string_set_status value_string_set_status = bt_value_string_set(directory, traces.at(i).c_str());
        switch (value_string_set_status) {
            case   BT_VALUE_STRING_SET_STATUS_OK: break;
            case   BT_VALUE_STRING_SET_STATUS_MEMORY_ERROR: fprintf(stderr, "\033[31;1mOut of Memory at at configuration\033[0m\n"); break;
            default: fprintf(stderr, "\033[31;1mHopefully never reached\033[0m\n");
        }
        
        bt_value_array_append_element_status array_append_element_status = bt_value_array_append_string_element(
            array_value,
            bt_value_string_get(directory)
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
        const bt_component_source *source_fs;

        std::string compName = "fs" + std::to_string(i);
        bt_graph_add_component_status add_component_status = bt_graph_add_source_component(
                graph,
                source_class_lttnglive,
                compName.c_str(),
                map_value,
                BT_LOGGING_LEVEL_WARNING,
                &source_fs
            );
            switch (add_component_status) {
                case BT_GRAPH_ADD_COMPONENT_STATUS_OK: break; printf("\033[32;1mSuccess\033[0m\n"); break;
                case BT_GRAPH_ADD_COMPONENT_STATUS_MEMORY_ERROR: fprintf(stderr, "\033[31;1mOut of Memory\033[0m\n"); break;
                case BT_GRAPH_ADD_COMPONENT_STATUS_ERROR: fprintf(stderr, "\033[31;1mOther error\033[0m\n"); break;
                default: fprintf(stderr, "\033[31;1mHopefully never reached\033[0m\n");
            }
        sources_fs.push_back(source_fs);

        bt_value_put_ref(map_value);
        bt_value_put_ref(directory);
        bt_value_put_ref(array_value);
    }


    // printf("Add filter Component to Graph....."); fflush(stdout);
    const bt_component_filter *filter_muxer;
    {
        bt_value *map_value = bt_value_map_create();
        bt_graph_add_component_status add_component_status = bt_graph_add_filter_component(
            graph,
            filter_class_muxer,
            "muxer",
            map_value,
            BT_LOGGING_LEVEL_WARNING,
            &filter_muxer
        );
        switch (add_component_status) {
            case BT_GRAPH_ADD_COMPONENT_STATUS_OK: break; printf("\033[32;1mSuccess\033[0m\n"); break;
            case BT_GRAPH_ADD_COMPONENT_STATUS_MEMORY_ERROR: fprintf(stderr, "\033[31;1mOut of Memory\033[0m\n"); break;
            case BT_GRAPH_ADD_COMPONENT_STATUS_ERROR: fprintf(stderr, "\033[31;1mOther error\033[0m\n"); break;
            default: fprintf(stderr, "\033[31;1mHopefully never reached\033[0m\n");
        }

        bt_value_put_ref(map_value);
    }


    // printf("Add sink Component to Graph....."); fflush(stdout);
    const bt_component_sink *sink_details;
    {
        bt_value *map_value = bt_value_map_create();

        // const bt_component_class_sink *sink_class_details;
        // sink_class_details = bt_plugin_borrow_sink_component_class_by_name_const(plugin_text, "text");

        bt_graph_add_component_status add_component_status = bt_graph_add_sink_component(
            graph,
            sink_class_text,
            "pretty",
            map_value,
            BT_LOGGING_LEVEL_WARNING,
            &sink_details
        );
        switch (add_component_status) {
            case BT_GRAPH_ADD_COMPONENT_STATUS_OK: break; printf("\033[32;1mSuccess\033[0m\n"); break;
            case BT_GRAPH_ADD_COMPONENT_STATUS_MEMORY_ERROR: fprintf(stderr, "\033[31;1mOut of Memory\033[0m\n"); break;
            case BT_GRAPH_ADD_COMPONENT_STATUS_ERROR: fprintf(stderr, "\033[31;1mOther error\033[0m\n"); break;
            default: fprintf(stderr, "\033[31;1mHopefully never reached\033[0m\n");
        }

        bt_value_put_ref(map_value);
    }

    /***Connect Components***/
    // printf("Get output port fs/out and input port muxer/in and connect\n");
    size_t muxPort = 0;
    for (size_t i = 0; i < sources_fs.size(); i++) {
        for (size_t fsPort = 0; fsPort < bt_component_source_get_output_port_count(sources_fs.at(i)); fsPort++) {
            const bt_port_output *source_fs_port_out;
            source_fs_port_out = bt_component_source_borrow_output_port_by_index_const(sources_fs.at(i), fsPort);
            const bt_port_input *filter_mux_port_in;
            filter_mux_port_in = bt_component_filter_borrow_input_port_by_index_const(filter_muxer, muxPort++);
            
            const bt_connection *connectionlttngdetails;
            bt_graph_connect_ports_status connect_ports_status = bt_graph_connect_ports(graph, source_fs_port_out, filter_mux_port_in, &connectionlttngdetails);
            switch (connect_ports_status) {
                case BT_GRAPH_CONNECT_PORTS_STATUS_OK: break; printf("\033[32;1mSuccess\033[0m\n"); break;
                case BT_GRAPH_CONNECT_PORTS_STATUS_MEMORY_ERROR: fprintf(stderr, "\033[31;1mOut of memory\033[0m\n"); break;
                case BT_GRAPH_CONNECT_PORTS_STATUS_ERROR: fprintf(stderr, "\033[31;1mOther Error\033[0m\n"); break;
                default: fprintf(stderr, "\033[31;1mHopefully never reached\033[0m\n");
            }
        }
    }

    // printf("Get output port muxer/out and input port pretty/in\n");
    const bt_port_output *filter_mux_port_out;
    filter_mux_port_out = bt_component_filter_borrow_output_port_by_name_const(filter_muxer, "out");
    const bt_port_input *source_pretty_port_in;
    source_pretty_port_in = bt_component_sink_borrow_input_port_by_name_const(sink_details, "in");
    
    const bt_connection *connectionmuxerdetails;
    bt_graph_connect_ports_status connect_ports_status = bt_graph_connect_ports(graph, filter_mux_port_out, source_pretty_port_in, &connectionmuxerdetails);
    switch (connect_ports_status) {
        case BT_GRAPH_CONNECT_PORTS_STATUS_OK: break; printf("\033[32;1mSuccess\033[0m\n"); break;
        case BT_GRAPH_CONNECT_PORTS_STATUS_MEMORY_ERROR: fprintf(stderr, "\033[31;1mOut of memory\033[0m\n"); break;
        case BT_GRAPH_CONNECT_PORTS_STATUS_ERROR: fprintf(stderr, "\033[31;1mOther Error\033[0m\n"); break;
        default: fprintf(stderr, "\033[31;1mHopefully never reached\033[0m\n");
    }

    /***Run Graph**/
    bt_graph_run_status graph_status;
    do {
        graph_status = bt_graph_run(graph);
    } while (graph_status == BT_GRAPH_RUN_STATUS_AGAIN);
    switch (graph_status) {
        case BT_GRAPH_RUN_STATUS_OK: break; printf("\033[32;1m.....Success\033[0m\n"); break;
        case BT_GRAPH_RUN_STATUS_MEMORY_ERROR: fprintf(stderr, "\033[31;1m.....Out of memory\033[0m\n"); break;
        case BT_GRAPH_RUN_STATUS_ERROR: fprintf(stderr, "\033[31;1m.....Other Error\033[0m\n"); break;
        default: fprintf(stderr, "\033[31;1m.....Hopefully never reached\033[0m\n");
    }

    bt_plugin_put_ref(ctlFs);
    bt_plugin_put_ref(muxer);
    bt_plugin_put_ref(plugin_text);
    bt_graph_put_ref(graph);

    return 0;
}