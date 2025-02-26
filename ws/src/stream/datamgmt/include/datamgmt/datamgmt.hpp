#pragma once

#include <vector>
#include <optional>
#include <atomic>

#include <ipc/ipc-server.hpp>
#include "pipe/pipe.hpp"
#include "common.hpp"

struct Module {
    std::optional<std::thread> thread;
    Pipe pipe;
    std::atomic<bool> running;

    Module() {
        int p[2];
        getPipe(p);
        this->pipe = Pipe{
            .read = p[0],
            .write = p[1],
        };
        this->running = false;
        this->thread = {};
    }
};

enum Module_t {
    NODEOBSERVER,
    LASTOPTION
};