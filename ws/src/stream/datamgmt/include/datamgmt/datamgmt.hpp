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
    RELATIONMGMT,
    PROCESSOBSERVER,
    TASKORCHESTRATOR,
    TASKEXECUTOR,
    DATATRACER,
    LASTOPTION,
    MAIN
};

struct Module {
    std::optional<std::thread>          thread;
    std::map<Module_t, pipe_ns::Pipe>   pipes;
    std::atomic<bool>                   running;

    Module() {
        int p[2];
        pipe_ns::getPipe(p);
        this->pipes[MAIN] = pipe_ns::Pipe {
            .read   = p[0],
            .write  = p[1],
        };
        this->running   = false;
        this->thread    = {};
    }
};