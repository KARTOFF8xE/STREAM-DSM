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
#include "datamgmt/nodeandtopicobserver/nodeandtopicobserver.hpp"
#include "datamgmt/processobserver/processobserver.hpp"
#include "datamgmt/relationmgmt/relationmgmt.hpp"
#include "datamgmt/taskOrchestrator/taskOrchestrator.hpp"
#include "datamgmt/taskExecutor/taskExecutor.hpp"
#include "datamgmt/datatracer/datatracer.hpp"
#include "datamgmt/tasks.hpp"


void runModule(IpcServer &server, Module_t module_t, Module &module) {
    switch (module_t) {
        case NODEANDTOPICOBSERVER:
            if (module.thread.has_value() && module.thread.value().joinable()) {
                module.thread.value().join();
            }
            module.running.store(true);
            module.thread = std::thread(nodeAndTopicObserver, std::cref(server), module.pipes, std::ref(module.running));
            return;
        case DATATRACER:
            if (module.thread.has_value() && module.thread.value().joinable()) {
                module.thread.value().join();
            }
            module.running.store(true);
            module.thread = std::thread(datatracer::datatracer, std::cref(server), module.pipes, std::ref(module.running));
            return;
        default:
            std::cerr << "No matching function found" << std::endl;
            return;
    }
}


void runModule(Module_t module_t, Module &module) {
    switch (module_t) {
        case RELATIONMGMT:
            if (module.thread.has_value() && module.thread.value().joinable()) {
                module.thread.value().join();
            }
            module.running.store(true);
            module.thread = std::thread(relationMgmt::relationMgmt, module.pipes, std::ref(module.running));
            return;
        case PROCESSOBSERVER:
            if (module.thread.has_value() && module.thread.value().joinable()) {
                module.thread.value().join();
            }
            module.running.store(true);
            module.thread = std::thread(processObserver::processObserver, module.pipes, std::ref(module.running));
            return;
        default:
            std::cerr << "No matching function found" << std::endl;
            return;
    }
}

void runModule(IpcServer &server, Module_t module_t, Module &module, Tasks &tasks) {
    switch (module_t) {
        case TASKORCHESTRATOR:
            if (module.thread.has_value() && module.thread.value().joinable()) {
                module.thread.value().join();
            }
            module.running.store(true);
            module.thread = std::thread(taskOrchestrator::taskOrchestrator, std::cref(server), module.pipes, std::ref(module.running), std::ref(tasks));
            return;
        case TASKEXECUTOR:
            if (module.thread.has_value() && module.thread.value().joinable()) {
                module.thread.value().join();
            }
            module.running.store(true);
            module.thread = std::thread(taskExecutor::taskExecutor, std::cref(server), module.pipes, std::ref(module.running), std::ref(tasks));
            return;
        default:
            std::cerr << "No matching function found" << std::endl;
            return;
    }
}

int main() {
    IpcServer nodeAndTopicObsServer(1);
    IpcServer taskServer(4);
    IpcServer tracerServer(5);

    std::map<Module_t, Module> modules;
    for (int i = NODEANDTOPICOBSERVER; i < LASTOPTION; i++) {
        modules[static_cast<Module_t>(i)];
    }

    {
        int p[2];
        pipe_ns::getPipe(p);
        modules[NODEANDTOPICOBSERVER].pipes[RELATIONMGMT] = pipe_ns::Pipe {
            .read   = p[0],
            .write  = p[1],
        };
        modules[RELATIONMGMT].pipes[NODEANDTOPICOBSERVER] = pipe_ns::Pipe {
            .read   = p[0],
            .write  = p[1],
        };
    }
    {
        int p[2];
        pipe_ns::getPipe(p);
        modules[RELATIONMGMT].pipes[PROCESSOBSERVER] = pipe_ns::Pipe {
            .read   = p[0],
            .write  = p[1],
        };
        modules[PROCESSOBSERVER].pipes[RELATIONMGMT] = pipe_ns::Pipe {
            .read   = p[0],
            .write  = p[1],
        };
    }
    {
        int p[2];
        pipe_ns::getPipe(p);
        modules[RELATIONMGMT].pipes[TASKORCHESTRATOR] = pipe_ns::Pipe {
            .read   = p[0],
            .write  = p[1],
        };
        modules[TASKORCHESTRATOR].pipes[RELATIONMGMT] = pipe_ns::Pipe {
            .read   = p[0],
            .write  = p[1],
        };
    }


    runModule(nodeAndTopicObsServer, NODEANDTOPICOBSERVER, modules[NODEANDTOPICOBSERVER]);
        usleep(250);
    runModule(RELATIONMGMT, modules[RELATIONMGMT]);
        usleep(250);
    runModule(PROCESSOBSERVER, modules[PROCESSOBSERVER]);
        usleep(250);
    runModule(tracerServer, DATATRACER, modules[DATATRACER]);
        usleep(250);
    Tasks tasks;
    runModule(taskServer, TASKORCHESTRATOR, modules[TASKORCHESTRATOR], tasks);
        usleep(250);
    runModule(taskServer, TASKEXECUTOR, modules[TASKEXECUTOR], tasks);
        usleep(250);

    auto then = std::chrono::steady_clock::now();
    while (true) {
        bool receivedMessage = false;
        {
            Client newClient;
            std::optional<NodeRequest> request = nodeAndTopicObsServer.receiveNodeRequest(newClient.requestId, newClient.pid, false);
            if (request.has_value()) {
                NodeRequest payload = request.value();
                Client clientInfo {
                    .pid        = newClient.pid,
                    .requestId  = newClient.requestId,
                    .primaryKey = payload.primaryKey,
                    .updates    = payload.updates,
                };
                pipe_ns::writeT<Client>(modules[NODEANDTOPICOBSERVER].pipes[MAIN].write, clientInfo);
                
                singleTimeNodeResponse(nodeAndTopicObsServer, clientInfo, payload.primaryKey);
                if (!modules[NODEANDTOPICOBSERVER].running) {
                    runModule(nodeAndTopicObsServer, NODEANDTOPICOBSERVER, modules[NODEANDTOPICOBSERVER]);
                }

                receivedMessage = true;
            }
        }
        {
            Client newClient;
            std::optional<TopicRequest> request = nodeAndTopicObsServer.receiveTopicRequest(newClient.requestId, newClient.pid, false);
            if (request.has_value()) {
                TopicRequest payload = request.value();
                Client clientInfo {
                    .pid        = newClient.pid,
                    .requestId  = newClient.requestId,
                    .primaryKey = payload.primaryKey,
                    .updates    = payload.updates,
                };
                pipe_ns::writeT<Client>(modules[NODEANDTOPICOBSERVER].pipes[MAIN].write, clientInfo);

                singleTimeTopicResponse(nodeAndTopicObsServer, clientInfo, payload.primaryKey);
                if (!modules[NODEANDTOPICOBSERVER].running) {
                    runModule(nodeAndTopicObsServer, NODEANDTOPICOBSERVER, modules[NODEANDTOPICOBSERVER]);
                }

                receivedMessage = true;
            }
        }

        if (receivedMessage) continue;


        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now-then).count();
        auto sleepTime = 1000000 - elapsed;
        if (sleepTime > 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(sleepTime));
        }
        then = std::chrono::steady_clock::now();

        // TODO receive different messages as a server (subscriptions and unsubscriptions)
        // TODO handle threads that are running
    }
}