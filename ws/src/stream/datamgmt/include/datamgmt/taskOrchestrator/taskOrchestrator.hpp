#include <atomic>
#include <ipc/ipc-server.hpp>

#include "datamgmt/common.hpp"
#include "datamgmt/datamgmt.hpp"
#include "datamgmt/tasks.hpp"


namespace taskOrchestrator {

// TODO
void taskOrchestrator(const IpcServer &server, std::map<Module_t, pipe_ns::Pipe> pipes, std::atomic<bool> &running, Tasks &tasks);

}