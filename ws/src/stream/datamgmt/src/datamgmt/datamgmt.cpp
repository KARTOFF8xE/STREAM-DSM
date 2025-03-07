#include <string>
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>
#include <future>
#include <mutex>
#include <condition_variable>
#include <map>
#include <bits/stdc++.h>

#include <ipc/ipc-server.hpp>
#include <ipc/ipc-client.hpp>
#include <ipc/util.hpp>
#include <neo4j/node/node.hpp>
#include <curl/myCurl.hpp>

#include "pipe/pipe.hpp"
#include "datamgmt/datamgmt.hpp"
#include "datamgmt/nodeobserver/nodeobserver.hpp"



void runModule(IpcServer &server, Module_t module_t, Module &module) {
    switch (module_t) {
        case NODEOBSERVER:
            if (module.thread.has_value() && module.thread.value().joinable()){
                module.thread.value().join();
            }
            module.running.store(true);
            module.thread = std::thread(nodeObserver, std::cref(server), module.pipe.read, std::ref(module.running));
            return;
        default:
            std::cerr << "No matching function found" << std::endl;
            return;
    }
}

int main() {
    IpcServer server(1);

    std::map<Module_t, Module> modules;
    for (int i = NODEOBSERVER; i < LASTOPTION; i++) {
        modules[static_cast<Module_t>(i)];
    }

    while (true) {
        Client newClient;
        std::optional<NodeRequest> request = server.receiveNodeRequest(newClient.requestId, newClient.pid, false);
        
        if (request.has_value()) {
            // extract msg and send to thread (write to pipe)
            NodeRequest requestPayload = request.value();
            Client clientInfo {
                .pid        = newClient.pid,
                .requestId  = newClient.requestId,
                .primaryKey = requestPayload.primaryKey,
                .updates    = requestPayload.updates,
            };
            writeT<Client>(modules[NODEOBSERVER].pipe.write, clientInfo);
            
            singleTimeNodeResponse(server, clientInfo, requestPayload.primaryKey);
            if (!modules[NODEOBSERVER].running) {
                runModule(server, NODEOBSERVER, modules[NODEOBSERVER]);
            }
        }

        // TODO receive different messages as a server (subscriptions and unsubscriptions)
        // TODO handle threads that are running
    }
}