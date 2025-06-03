#include "continuous/sink.hpp"

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <optional>
#include <babeltrace2/babeltrace.h>
#include <influxdb/influxdb.hpp>
#include <curl/myCurl.hpp>
#include <neo4j/publisher/publisher.hpp>
#include <nlohmann/json.hpp>

#include <ipc/ipc-server.hpp>
#include <neo4j/publisher/publisher.hpp>

#include "continuous/participants.hpp"

static bt_component_class_initialize_method_status tracer_initialize(
        bt_self_component_sink *self_component_sink,
        bt_self_component_sink_configuration *,
        const bt_value *params, void *) {

    struct tracer *tracer = new struct tracer();

    bt_self_component_set_data(
        bt_self_component_sink_as_self_component(self_component_sink),
        tracer);

    bt_self_component_sink_add_input_port(self_component_sink,
        "in", NULL, NULL);

    return BT_COMPONENT_CLASS_INITIALIZE_METHOD_STATUS_OK;
}

static void tracer_finalize(bt_self_component_sink *self_component_sink) {
    struct tracer *tracer = (struct tracer *)bt_self_component_get_data(
        bt_self_component_sink_as_self_component(self_component_sink));
 
    free(tracer);
}

static bt_component_class_sink_graph_is_configured_method_status
tracer_graph_is_configured(bt_self_component_sink *self_component_sink) {
    struct tracer *tracer = (struct tracer *)bt_self_component_get_data(
        bt_self_component_sink_as_self_component(self_component_sink));
 
    bt_self_component_port_input *in_port =
        bt_self_component_sink_borrow_input_port_by_index(
            self_component_sink, 0);
 
    bt_message_iterator_create_from_sink_component(self_component_sink,
        in_port, &tracer->message_iterator);
 
    return BT_COMPONENT_CLASS_SINK_GRAPH_IS_CONFIGURED_METHOD_STATUS_OK;
}

static void publish(bt_self_component_sink *self_component_sink, const bt_message *message) {
    struct tracer *tracer = (struct tracer *)bt_self_component_get_data(
        bt_self_component_sink_as_self_component(self_component_sink));

    sharedMem::TraceMessage msg = extractTracedMessage(self_component_sink, message);
}

void sendPubDataToTimeSeries(std::unordered_map<u_int64_t, u_int32_t> &pubRates) {
    std::vector<influxDB::ValuePairs> values;
    time_t timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    for (auto &pubRate : pubRates) {
        std::string payload = publisher::getprimKeyByPubHandle(pubRate.first);
        std::string responseNeo4J = curl::push(payload, curl::NEO4J);
        nlohmann::json data = nlohmann::json::parse(responseNeo4J);
        if (!data["results"].empty() &&
            !data["results"][0]["data"].empty() && 
            !data["results"][0]["data"][0]["row"].empty()) {
            influxDB::ValuePairs value {
                .attribute  = influxDB::PUBLISHINGRATE,
                .primaryKey = data["results"][0]["data"][0]["row"][0].get<std::string>(),
                .timestamp  = timestamp,
                .value      = double(pubRate.second)
            };
            values.push_back(value);
        }
    }

    std::string payload = influxDB::createPayloadMultipleVal(values);
    std::string response = curl::push(payload, curl::INFLUXDB_WRITE);
    if (response != "") std::cout << response << std::endl;
    pubRates.clear();
}

bt_component_class_sink_consume_method_status tracer_consume(
        bt_self_component_sink *self_component_sink) {
    struct tracer *tracer = (struct tracer *)bt_self_component_get_data(
        bt_self_component_sink_as_self_component(self_component_sink));

    bt_message_array_const messages;
    uint64_t message_count;
    bt_message_iterator_next_status next_status =
        bt_message_iterator_next(tracer->message_iterator, &messages,
            &message_count);
    switch (next_status) {
        case BT_MESSAGE_ITERATOR_NEXT_STATUS_END:
            bt_message_iterator_put_ref(tracer->message_iterator);
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
            publish(self_component_sink, message);
        }
        if (bt_message_get_type(message) == BT_MESSAGE_TYPE_STREAM_BEGINNING) {
            tracer->nrOfTraces++;
        }
        if (bt_message_get_type(message) == BT_MESSAGE_TYPE_STREAM_END && --tracer->nrOfTraces == 0) {
            if (!tracer->publishingRate.empty()) sendPubDataToTimeSeries(tracer->publishingRate);
        }
 
        bt_message_put_ref(message);
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now-tracer->lastTick);
    }

    return BT_COMPONENT_CLASS_SINK_CONSUME_METHOD_STATUS_OK;
}

BT_PLUGIN_MODULE();

BT_PLUGIN(continuous);

BT_PLUGIN_AUTHOR("KARTOFF8xE");
BT_PLUGIN_DESCRIPTION("This plugin gatheres continuous ROS2 information and sends it to time-series database.");
BT_PLUGIN_VERSION(0, 0, 1, "dev");

BT_PLUGIN_SINK_COMPONENT_CLASS(output, tracer_consume);

BT_PLUGIN_SINK_COMPONENT_CLASS_INITIALIZE_METHOD(output,
    tracer_initialize);
BT_PLUGIN_SINK_COMPONENT_CLASS_FINALIZE_METHOD(output, tracer_finalize);
BT_PLUGIN_SINK_COMPONENT_CLASS_GRAPH_IS_CONFIGURED_METHOD(output,
    tracer_graph_is_configured);