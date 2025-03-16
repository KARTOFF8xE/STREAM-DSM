#pragma once

#include <vector>
#include <optional>
#include <atomic>
#include <thread>
#include <map>

#include <ipc/ipc-server.hpp>
#include "pipe/pipe.hpp"
#include "common.hpp"


enum Module_t {
    NODEANDTOPICOBSERVER,
    LASTOPTION,
    MAIN
};

struct Module {
    std::optional<std::thread> thread;
    std::map<Module_t, Pipe>   pipes;
    std::atomic<bool>          running;

    Module() {
        int p[2];
        getPipe(p);
        this->pipes[MAIN] = Pipe{
            .read   = p[0],
            .write  = p[1],
        };
        this->running   = false;
        this->thread    = {};
    }
};