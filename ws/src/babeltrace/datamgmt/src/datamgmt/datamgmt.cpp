#include <string>
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <map>

#include <ipc/ipc-server.hpp>

#include "pipe/pipe.hpp"
#include "datamgmt/datamgmt.hpp"

std::mutex m;
std::condition_variable cv;

void procObserver(int pipe) {
    // TODO gather a list with all Node-PIDs (neo4j query)
    // TODO needs a communication with tracer (subscribe Events)
    
    // TODO loop to read pipe and check for updates
        // TODO if no more subscribers, exit loop

    // TODO unsubscribe Events
}

void runModule(Module_t module_t, Module &module) {
    switch (module_t) {
        case PROCOBSERVER:
            module.thread = std::thread(procObserver, module.pipe.read);
            return;
        default:
            std::cerr << "No matching function found" << std::endl;
            return;
    }
}

int main() {
    IpcServer server(0);

    std::map<Module_t, Module> modules;
    for (int i = PROCOBSERVER; i < DUMMY; i++) {
        modules[static_cast<Module_t>(i)] = Module{
            .thread = {}
        };
    }

    while (true) {
        Client newClient;
        std::optional<ProcessRequest> request = server.receiveProcessRequest(newClient.requestId, newClient.pid, false);
        if (request.has_value()) {
            if (!modules[PROCOBSERVER].thread.has_value()) {    // create thread if not existing
                runModule(PROCOBSERVER, modules[PROCOBSERVER]);
            }
            
            // extract msg and send to thread (write to pipe)
            ProcessRequest requestPayload = request.value();
            Data clientInfo {
                .client = Client{
                    .pid = requestPayload.pid,
                },
                .subscribed = requestPayload.updates
            };

            writeT<Data>(modules[PROCOBSERVER].pipe.write, clientInfo);
        }
        // TODO receive different messages as a server (subscriptions and unsubscriptions)
        // TODO handle threads that are running
    }
}