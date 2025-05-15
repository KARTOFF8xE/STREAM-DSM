#include <atomic>
#include <ipc/ipc-server.hpp>

#include "datamgmt/common.hpp"
#include "datamgmt/datamgmt.hpp"


namespace taskExecutor {

// TODO
void taskExecutor(const IpcServer &server, std::map<Module_t, Pipe> pipes, std::atomic<bool> &running);

}