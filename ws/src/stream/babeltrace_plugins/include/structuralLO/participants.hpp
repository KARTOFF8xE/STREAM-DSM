#pragma once

#include <babeltrace2/babeltrace.h>
#include <ipc/sharedMem.hpp>


// TODO
sharedMem::TraceMessage extractTracedMessage(bt_self_component_sink *self_component_sink, const bt_message *message);