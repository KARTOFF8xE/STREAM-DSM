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
#include <neo4j/node/node.hpp>
#include <curl/myCurl.hpp>

#include "pipe/pipe.hpp"
#include "datamgmt/datamgmt.hpp"

/**
 * Make ProcObserver to NodeObserver
 * [x] BabelTrace_Plugin sends updates to Node creation
 * [x] BabelTrace_Plugin sends updates to pub creation
 * [x] BabelTrace_Plugin sends updates to sub creation
 * [x] BabelTrace_Plugin sends updates to service creation
 * [x] BabelTrace_Plugin sends updates to client creation
 * [x] enable and disable Node updates
 * 
 * [ ] make a single-time response (just Query neo4j and return stuff)
 * [ ] make an update response
 *      [ ] make list of receivers
 *          [ ] every receiver has a list of nodes he wants to observe
 *          [ ] every receiver has a list of stuff (topics, services) he wants to ignore
 *      [!] receivers (and Node) are send by "main module" and received by module
 *      [x] if there are no more receivers, the
 */

void nodeObserver(int pipe_r, std::atomic<bool> &running) {
    std::cout << "started procObserver" << std::endl;

    std::vector<Client> clients;
    std::vector<pid_t> pids;
    
    IpcClient ipcClient(2);
    requestId_t requestId;
    {
        NodeSwitchRequest msg = NodeSwitchRequest{
            .updates = true
        };
        ipcClient.sendNodeSwitchRequest(msg, requestId, false);
    }
    
    Client client;
    do {
        std::optional<NodeSwitchResponse> response = ipcClient.receiveNodeSwitchResponse(false);
        if (response.has_value()) {
            switch (response.value().type)
            {
                case NODE:
                    pids.push_back(response.value().pid);
                    std::cout << "Received new Node with PID: " << response.value().pid <<
                        "\n\talive: " << response.value().alive <<
                        "\n\talive_changeTime: " << response.value().aliveChangeTime <<
                        "\n\tid (primaryKey): " << response.value().primary_key <<
                        "\n\tbootcounter: " << response.value().bootCounter <<
                    std::endl;
                    break;
                case PUB:
                    std::cout << "New Pub: " << response.value().pub << std::endl;
                    break;
                case SUB:
                    std::cout << "New Sub: " << response.value().sub << std::endl;
                    break;
                case SERVICE:
                    std::cout << "New Service: " << response.value().service << std::endl;
                    break;
                case CLIENT:
                    std::cout << "New Client: " << response.value().client << std::endl;
                    break;
                default:
                    std::cout << "Unknown Type" << std::endl;
                    break;
            }
        }

        int ret = readT<Client>(pipe_r, client);
        if (ret != -1) {
            std::cout << "pid: " << client.pid << "\tupdates: " << client.updates << std::endl;
            if (client.updates) {
                if (std::find(clients.begin(), clients.end(), client) == clients.end()) {
                    clients.push_back(client);
                }
            } else if (std::find(clients.begin(), clients.end(), client) != clients.end()) {
                clients.erase(find(clients.begin(), clients.end(), client));
            }
            std::cout << "nrOf supped Clients: " << clients.size() << std::endl;
        }

        for (const pid_t &pid : pids) {
            if (kill(pid, 0) != 0) {
                std::cout << pid << " died." << std::endl;
                pids.erase(find(pids.begin(), pids.end(), pid));
                curl::push(node::getPayloadSetOffline(pid));
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    } while(!clients.empty());
    std::cout << "empty now" << std::endl;

    // TODO gather a list with all Node-PIDs (neo4j query)
    // TODO needs a communication with tracer (subscribe Events)
    
    // TODO loop to read pipe and check for updates
        // TODO if no more subscribers, exit loop

    // TODO unsubscribe Events

    NodeSwitchRequest msg = NodeSwitchRequest{
        .updates = false
    };
    ipcClient.sendNodeSwitchRequest(msg, requestId, false);

    running.store(false);
}

void singleTimeNodeResponse(Client client, primaryKey_t primaryKey) {
    std::string response = curl::push(node::getPayloadRequestByPrimaryKey(primaryKey));
    std::cout << response << std::endl;
}

void runModule(Module_t module_t, Module &module) {
    switch (module_t) {
        case NODEOBSERVER:
            if (module.thread.has_value() && module.thread.value().joinable()){
                module.thread.value().join();
            }
            module.running.store(true);
            module.thread = std::thread(nodeObserver, module.pipe.read, std::ref(module.running));
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
                .pid = newClient.pid,
                .requestId = newClient.requestId,
                .updates = requestPayload.updates,
            };
            writeT<Client>(modules[NODEOBSERVER].pipe.write, clientInfo);
            
            singleTimeNodeResponse(clientInfo, primaryKey_t(14));
            // singleTimeNodeResponse(clientInfo, requestPayload.primaryKey);
            if (!modules[NODEOBSERVER].running) {
                runModule(NODEOBSERVER, modules[NODEOBSERVER]);
            }
        }

        // TODO receive different messages as a server (subscriptions and unsubscriptions)
        // TODO handle threads that are running
    }
}