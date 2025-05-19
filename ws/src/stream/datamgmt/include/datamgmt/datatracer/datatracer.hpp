#pragma once

#include <nlohmann/json.hpp>
#include <map>

#include <ipc/ipc-server.hpp>

#include "pipe/pipe.hpp"
#include "datamgmt/datamgmt.hpp"



namespace datatracer {

// TODO
void datatracer(const IpcServer &server,  std::map<Module_t, pipe_ns::Pipe> pipes, std::atomic<bool> &running) ;

}