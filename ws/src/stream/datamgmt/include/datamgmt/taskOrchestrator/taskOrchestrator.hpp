#include <atomic>
#include <ipc/ipc-server.hpp>

#include "datamgmt/common.hpp"
#include "datamgmt/datamgmt.hpp"


namespace taskOrchestrator {

// TODO
void taskOrchestrator(const IpcServer &server, std::map<Module_t, Pipe> pipes, std::atomic<bool> &running);

}