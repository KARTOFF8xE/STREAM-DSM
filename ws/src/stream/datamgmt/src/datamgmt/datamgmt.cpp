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
#include <sys/mman.h>

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
#include "datamgmt/common.hpp"


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
    gsRunning = false;
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
    // IpcServer nodeAndTopicObsServer(1);
    // IpcServer taskServer(4);
    // IpcServer tracerServer(5);
    IpcServer ipcServer(1);

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

    sighandler_t stdHandler = std::signal(SIGINT, handle_sigint);
    gsRunning = true;
    runModule(ipcServer, NODEANDTOPICOBSERVER, modules[NODEANDTOPICOBSERVER]);
        usleep(250);
    runModule(RELATIONMGMT, modules[RELATIONMGMT]);
        usleep(250);
    runModule(PROCESSOBSERVER, modules[PROCESSOBSERVER]);
        usleep(250);
    runModule(ipcServer, DATATRACER, modules[DATATRACER]);
        usleep(250);
    Tasks tasks;
    runModule(ipcServer, TASKORCHESTRATOR, modules[TASKORCHESTRATOR], tasks);
        usleep(250);
    runModule(TASKEXECUTOR, modules[TASKEXECUTOR], tasks);
        usleep(250);
    
    modules[TASKEXECUTOR].thread->join();
    modules[TASKORCHESTRATOR].thread->join();
    modules[DATATRACER].thread->join();
    modules[NODEANDTOPICOBSERVER].thread->join();
    modules[PROCESSOBSERVER].thread->join();
    modules[RELATIONMGMT].thread->join();
    std::signal(SIGINT, stdHandler);

    // clearMsgQueue(1);
    // clearMsgQueue(4);
    // clearMsgQueue(5);
    clearMsgQueue(0);

    std::cout << "finalized" << std::endl;
}