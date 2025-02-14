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

#include "pipe/pipe.hpp"
#include "datamgmt/datamgmt.hpp"

std::mutex m;
std::condition_variable cv;

void procObserver(int pipe_r, std::atomic<bool> &running) {
    std::cout << "started procObserver" << std::endl;

    std::vector<Client> clients;
    std::vector<pid_t> pids;
    
    IpcClient ipcClient(2);
    requestId_t requestId;
    {
        ProcSwitchRequest msg = ProcSwitchRequest{
            .updates = true
        };
        ipcClient.sendProcSwitchRequest(msg, requestId, false);
    }
    
    Client client;
    do {
        std::optional<ProcSwitchResponse> response = ipcClient.receiveProcSwitchResponse(false);
        if (response.has_value()) {
            pids.push_back(response.value().pid);
            std::cout << "Received new Node with PID: " << response.value().pid << std::endl;
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
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    } while(!clients.empty());
    std::cout << "empty now" << std::endl;

    // TODO gather a list with all Node-PIDs (neo4j query)
    // TODO needs a communication with tracer (subscribe Events)
    
    // TODO loop to read pipe and check for updates
        // TODO if no more subscribers, exit loop

    // TODO unsubscribe Events

    ProcSwitchRequest msg = ProcSwitchRequest{
        .updates = false
    };
    ipcClient.sendProcSwitchRequest(msg, requestId, false);

    running.store(false);
}

void runModule(Module_t module_t, Module &module) {
    switch (module_t) {
        case PROCOBSERVER:
            if (module.thread.has_value() && module.thread.value().joinable()){
                module.thread.value().join();
            }
            module.running.store(true);
            module.thread = std::thread(procObserver, module.pipe.read, std::ref(module.running));
            return;
        default:
            std::cerr << "No matching function found" << std::endl;
            return;
    }
}

int main() {
    IpcServer server(1);

    std::map<Module_t, Module> modules;
    for (int i = PROCOBSERVER; i < LASTOPTION; i++) {
        modules[static_cast<Module_t>(i)];
    }

    while (true) {
        Client newClient;
        std::optional<ProcessRequest> request = server.receiveProcessRequest(newClient.requestId, newClient.pid, false);
        
        if (request.has_value()) {
            // extract msg and send to thread (write to pipe)
            ProcessRequest requestPayload = request.value();
            Client clientInfo {
                .pid = newClient.pid,
                .requestId = newClient.requestId,
                .updates = requestPayload.updates,
            };
            writeT<Client>(modules[PROCOBSERVER].pipe.write, clientInfo);

            if (!modules[PROCOBSERVER].running) {
                runModule(PROCOBSERVER, modules[PROCOBSERVER]);
            }
        }

        // TODO receive different messages as a server (subscriptions and unsubscriptions)
        // TODO handle threads that are running
    }
}