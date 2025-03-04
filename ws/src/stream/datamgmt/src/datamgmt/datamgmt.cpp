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

#include <nlohmann/json.hpp>

#include <ipc/ipc-server.hpp>
#include <ipc/ipc-client.hpp>
#include <ipc/util.hpp>
#include <neo4j/node/node.hpp>
#include <curl/myCurl.hpp>

#include "pipe/pipe.hpp"
#include "datamgmt/datamgmt.hpp"

using json = nlohmann::json;

void nodeObserver(IpcServer &server, int pipe_r, std::atomic<bool> &running) {
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
        int ret = readT<Client>(pipe_r, client);
        if (ret != -1) {
            std::cout << "pid:   "  << client.pid <<
                "\n\trequestId:  "  << client.requestId <<
                "\n\tprimaryKey: "  << client.primaryKey <<
                "\n\tupdates:    "  << client.updates <<
                std::endl;

            if (client.updates) {
                bool contains = false;
                for (Client item : clients) {
                    if (item.pid == client.pid &&
                        item.requestId == client.requestId
                    ) { contains = true; break; }
                }
                if (!contains) clients.push_back(client);
            } else {
                for (size_t i = 0; i < clients.size(); i++) {
                    if (clients[i].pid == client.pid &&
                        clients[i].requestId == client.pid
                    ) {
                        clients.erase(clients.begin() + i * sizeof(Client));
                        break;
                    }
                }
            }
            std::cout << "nrOf supped Clients: " << clients.size() << std::endl;
        }

        /*
        for (const pid_t &pid : pids) {
            if (kill(pid, 0) != 0) {
                std::cout << pid << " died." << std::endl;
                pids.erase(find(pids.begin(), pids.end(), pid));
                curl::push(node::getPayloadSetOffline(pid));
            }
        }
        */

        {
            std::optional<NodeResponse> response = ipcClient.receiveNodeResponse(false);
            if (response.has_value()) {
                NodeResponse payload = response.value();
                for (Client client : clients) {
                    if (client.primaryKey == payload.primaryKey) {
                        server.sendNodeResponse(payload, client.pid, false);
                    }
                }
            }
        }
        {
            std::optional<NodePublishersToUpdate> response = ipcClient.receiveNodePublishersToUpdate(false);
            if (response.has_value()) {
                NodePublishersToUpdate payload = response.value();
                for (Client client : clients) {
                    if (client.primaryKey == payload.primaryKey) {
                        server.sendNodePublishersToUpdate(payload, client.pid, false);
                    }
                }
            }
        }
        {
            std::optional<NodeSubscribersToUpdate> response = ipcClient.receiveNodeSubscribersToUpdate(false);
            if (response.has_value()) {
                NodeSubscribersToUpdate payload = response.value();
                for (Client client : clients) {
                    if (client.primaryKey == payload.primaryKey) {
                        server.sendNodeSubscribersToUpdate(payload, client.pid, false);
                    }
                }
            }
        }
        {
            std::optional<NodeIsServerForUpdate> response = ipcClient.receiveNodeIsServerForUpdate(false);
            if (response.has_value()) {
                NodeIsServerForUpdate payload = response.value();
                for (Client client : clients) {
                    if (client.primaryKey == payload.primaryKey) {
                        server.sendNodeIsServerForUpdate(payload, client.pid, false);
                    }
                    if (client.primaryKey == payload.clientNodeId) {
                        NodeIsClientOfUpdate msg {
                            .primaryKey     = payload.clientNodeId,
                            .serverNodeId   = payload.primaryKey,
                            .isUpdate       = true,
                        };
                        util::parseString(msg.srvName, payload.srvName);

                        server.sendNodeIsClientOfUpdate(msg, client.pid, false);
                    }
                }
            }
        }
        {
            std::optional<NodeIsClientOfUpdate> response = ipcClient.receiveNodeIsClientOfUpdate(false);
            if (response.has_value()) {
                NodeIsClientOfUpdate payload = response.value();
                for (Client client : clients) {
                    if (client.primaryKey == payload.primaryKey) {
                        server.sendNodeIsClientOfUpdate(payload, client.pid, false);
                    }
                    if (client.primaryKey == payload.serverNodeId) {
                        NodeIsServerForUpdate msg {
                            .primaryKey     = payload.serverNodeId,
                            .clientNodeId   = payload.primaryKey,
                            .isUpdate       = true,
                        };
                        util::parseString(msg.srvName, payload.srvName);

                        server.sendNodeIsServerForUpdate(msg, client.pid, false);
                    }
                }
            }
        }

        // std::this_thread::sleep_for(std::chrono::milliseconds(250));
    } while(!clients.empty());
    std::cout << "empty now" << std::endl;

    NodeSwitchRequest msg = NodeSwitchRequest{
        .updates = false
    };
    ipcClient.sendNodeSwitchRequest(msg, requestId, false);

    running.store(false);
}

void singleTimeNodeResponse(IpcServer &server, Client client, primaryKey_t primaryKey) {
    std::string response = curl::push(node::getPayloadRequestByPrimaryKey(primaryKey));

    json payload = json::parse(response);
    json row = payload["results"][0]["data"][0]["row"];

    json requestedNode;
    std::vector<NodePublishersToUpdate>     pubs;
    std::vector<NodeSubscribersToUpdate>    subs;
    std::vector<NodeIsServerForUpdate>      isServerFor;
    std::vector<NodeIsClientOfUpdate>       isClientOf;
    for (const json & item: row) {
        if (item.contains("name")) {
            requestedNode = item;
            continue;
        }

        size_t counter = 0;
        while (item[counter].contains("relationship")) {
            if (item[counter].contains("relationship")) {
                if (item[counter]["relationship"] == "publishes_to") {
                    for (const json & node: item[counter]["nodes"]) {
                        pubs.push_back(
                            NodePublishersToUpdate {
                                .primaryKey     = primaryKey,
                                .publishesTo    = node["id"],
                                .isUpdate       = false,
                            }
                        );
                    }
                }
                if (item[counter]["relationship"] == "subscription") {
                    for (const json & node: item[counter]["nodes"]) {
                        subs.push_back(
                            NodeSubscribersToUpdate {
                                .primaryKey     = primaryKey,
                                .subscribesTo   = node["id"],
                                .isUpdate       = false,
                            }
                        );
                    }
                }
                if (item[counter]["relationship"] == "request") {
                    for (const json & node: item[counter]["nodes"]) {
                        if (node["direction"] == "incoming") {
                            NodeIsServerForUpdate tmp = NodeIsServerForUpdate {
                            .primaryKey     = primaryKey,
                            .clientNodeId   = node["id"],
                            .isUpdate       = false,
                            };
                            util::parseString(tmp.srvName, node["servername"].get<std::string>());

                            isServerFor.push_back(tmp);
                        } else {
                            NodeIsClientOfUpdate tmp = NodeIsClientOfUpdate {
                                .primaryKey     = primaryKey,
                                .serverNodeId   = node["id"],
                                .isUpdate       = false,
                            };
                            util::parseString(tmp.srvName, node["servername"].get<std::string>());

                            isClientOf.push_back(tmp);
                        }
                    }
                }
            }
            counter++;
        }
    }

    NodeResponse nodeResponse {
        .primaryKey         = primaryKey,
        .alive              = requestedNode["state"] > 0,
        .aliveChangeTime    = requestedNode["stateChangeTime"],
        .bootCount          = requestedNode["bootcounter"],
        .pid                = requestedNode["pid"],
        .nrOfInitialUpdates = pubs.size() + subs.size() + isClientOf.size() + isServerFor.size(),
    };
    util::parseString(nodeResponse.name, requestedNode["name"].get<std::string>());
    server.sendNodeResponse(nodeResponse, client.pid);

    for (NodePublishersToUpdate item : pubs) {
        server.sendNodePublishersToUpdate(item, client.pid);
    }
    for (NodeSubscribersToUpdate item : subs) {
        server.sendNodeSubscribersToUpdate(item, client.pid);
    }
    for (NodeIsClientOfUpdate item : isClientOf) {
        server.sendNodeIsClientOfUpdate(item, client.pid);
    }
    for (NodeIsServerForUpdate item : isServerFor) {
        server.sendNodeIsServerForUpdate(item, client.pid);
    }
}

void runModule(IpcServer &server, Module_t module_t, Module &module) {
    switch (module_t) {
        case NODEOBSERVER:
            if (module.thread.has_value() && module.thread.value().joinable()){
                module.thread.value().join();
            }
            module.running.store(true);
            module.thread = std::thread(nodeObserver, std::ref(server), module.pipe.read, std::ref(module.running));
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