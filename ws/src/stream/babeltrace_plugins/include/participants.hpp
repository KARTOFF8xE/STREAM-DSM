#pragma once

#include <babeltrace2/babeltrace.h>
#include <ipc/sharedMem.hpp>


// TODO
sharedMem::TraceMessage extractTracedMessage(const bt_message *message);