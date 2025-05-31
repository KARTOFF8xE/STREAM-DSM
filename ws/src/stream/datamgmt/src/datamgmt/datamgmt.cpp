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
#include <sys/ipc.h>
#include <sys/msg.h>

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


struct MsgBuf {
    long mtype;
    char mtext[1024];
};

void clearMsgQueue(int msgQueueId) {
    MsgBuf msg;
    while (true) {
        ssize_t res = msgrcv(msgQueueId, &msg, sizeof(msg.mtext), 0, IPC_NOWAIT);
        if (res == -1) {
            if (errno == ENOMSG) {
                break;
            } else {
                std::cerr << "Error while clearing message queue: " << strerror(errno) << std::endl;
                break;
            }
        }
    }
}

void handle_sigint(int) {
    clearMsgQueue(1);
    clearMsgQueue(4);
    clearMsgQueue(5);
    exit(EXIT_SUCCESS);
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

void runModule(IpcServer &server, Module_t module_t, Module &module, Tasks &tasks) {
    // switch (module_t) {
    //     case TASKORCHESTRATOR:
            if (module.thread.has_value() && module.thread.value().joinable()) {
                module.thread.value().join();
            }
            module.running.store(true);
            module.thread = std::thread(taskOrchestrator::taskOrchestrator, std::cref(server), module.pipes, std::ref(module.running), std::ref(tasks));
            return;
    //     default:
    //         std::cerr << "No matching function found" << std::endl;
    //         return;
    // }
}

void runModule(Module_t module_t, Module &module, Tasks &tasks) {
    // switch (module_t) {
    //     case TASKEXECUTOR:
            if (module.thread.has_value() && module.thread.value().joinable()) {
                module.thread.value().join();
            }
            module.running.store(true);
            module.thread = std::thread(taskExecutor::taskExecutor, module.pipes, std::ref(module.running), std::ref(tasks));
            return;
        // default:
        //     std::cerr << "No matching function found" << std::endl;
        //     return;
    // }
}

int main() {
    std::signal(SIGINT, handle_sigint);

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
    {
        int p[2];
        pipe_ns::getPipe(p);
        modules[PROCESSOBSERVER].pipes[NODEANDTOPICOBSERVER] = pipe_ns::Pipe {
            .read   = p[0],
            .write  = p[1],
        };
        modules[NODEANDTOPICOBSERVER].pipes[PROCESSOBSERVER] = pipe_ns::Pipe {
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
    runModule(TASKEXECUTOR, modules[TASKEXECUTOR], tasks);
        usleep(250);

    auto then = std::chrono::steady_clock::now();
    while (true) {

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now-then).count();
        auto sleepTime = 1000000000000000 - elapsed;
        if (sleepTime > 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(sleepTime));
        }
        then = std::chrono::steady_clock::now();

        // TODO receive different messages as a server (subscriptions and unsubscriptions)
        // TODO handle threads that are running
    }
}