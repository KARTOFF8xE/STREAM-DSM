#pragma once

#include <vector>
#include <optional>

#include <ipc/ipc-server.hpp>
#include "common.hpp"

struct Pipe {
    int read;
    int write;
};

struct Module {
    Pipe pipe;
    std::optional<std::thread> thread;
};

enum Module_t {
    PROCOBSERVER,
    DUMMY  
};