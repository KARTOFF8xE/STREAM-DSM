#include <babeltrace2/babeltrace.h>

#include "ipc/sharedMem.hpp"

#include <memory>
#include <unordered_map>
#include <thread>
#include <chrono>
using namespace std::chrono_literals;


struct tracer {
    bt_message_iterator *message_iterator;
    
    sharedMem::SHMChannel<sharedMem::TraceMessage> channel;
    std::unordered_map<u_int64_t, u_int32_t>    timerInit;
    
    std::chrono::_V2::steady_clock::time_point lastTick = std::chrono::steady_clock::now();

    tracer(const std::string& name)
        : channel(name.c_str(), false)
    {}
};

/**
 * @brief Initializes the tracer component, sets the "topic" parameter, and adds an input port to the sink component.
 *
 * @param self_component_sink The component being initialized.
 * @param params The configuration parameters.
 * 
 * @return BT_COMPONENT_CLASS_INITIALIZE_METHOD_STATUS_OK on success.
 */
static bt_component_class_initialize_method_status tracer_initialize(
        bt_self_component_sink *self_component_sink,
        bt_self_component_sink_configuration *,
        const bt_value *params, void *);

/**
 * @brief Finalizes the tracer component and frees allocated memory.
 *
 * @param self_component_sink The component to finalize.
 */
static void tracer_finalize(bt_self_component_sink *self_component_sink);

/**
 * @brief Configures the tracer component's input port and creates a message iterator.
 *
 * @param self_component_sink The component to configure.
 * 
 * @return BT_COMPONENT_CLASS_SINK_GRAPH_IS_CONFIGURED_METHOD_STATUS_OK on success.
 */
static bt_component_class_sink_graph_is_configured_method_status
tracer_graph_is_configured(bt_self_component_sink *self_component_sink);

/**
 * @brief Retrieves and prints the value of a structure field member based on its type.
 *
 * @param structure_field The structure field containing the member.
 * @param member The structure member whose value is to be retrieved.
 */
void get_value(
    const bt_field *structure_field,
    const bt_field_class_structure_member *member
    );

/**
 * @brief Builds the payload used to query Graph-DB.
 *
 * @return The payload.
 */
void analyzeField(const bt_field *field);

/**
 * @brief Analyzes a field and prints the values of its structure members if it's a structure type. Else an Error will occure
 *
 * @param field The field to analyze.
 */
static void publish(bt_self_component_sink *self_component_sink, const bt_message *message);

/**
 * @brief Consumes messages from the iterator and publishes events if applicable.
 *
 * @param self_component_sink The component from which to consume messages.
 * 
 * @return The status of the consume method, indicating success or failure.
 */
bt_component_class_sink_consume_method_status tracer_consume(
        bt_self_component_sink *self_component_sink);