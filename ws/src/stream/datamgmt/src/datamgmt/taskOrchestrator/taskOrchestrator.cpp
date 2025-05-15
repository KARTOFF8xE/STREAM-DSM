#include <iostream>
#include <vector>
#include <nlohmann/json.hpp>
#include <chrono>
using namespace std::chrono_literals;

#include <ipc/ipc-client.hpp>
#include <ipc/util.hpp>
#include <curl/myCurl.hpp>
#include <neo4j/node/node.hpp>
#include <neo4j/topic/topic.hpp>

#include "datamgmt/taskOrchestrator/taskOrchestrator.hpp"
#include "pipe/pipe.hpp"


using json = nlohmann::json;


namespace taskOrchestrator {


bool receiveIPCClientRequest(const IpcServer &ipcServer, SingleStandardInformationRequest &receivedRequest) {
    std::optional<StandardSingleAttributesRequest> response =
        ipcServer.receiveStandardSingleAttributesRequest(receivedRequest.requestID, receivedRequest.pid, false);

    if (response.has_value()) {
        std::cout << "(1) received Request:" <<
                    "\treceivedRequest.requestID: " << receivedRequest.requestID <<
                    "\treceivedRequest.pid: " << receivedRequest.pid <<
            std::endl;
        receivedRequest.payload = response.value();

        return true;
    }

    return false;
}


void taskOrchestrator(const IpcServer &server, std::map<Module_t, Pipe> pipes, std::atomic<bool> &running) {
    std::cout << "started taskOrchestrator" << std::endl;
    int pipe_r = pipes[RELATIONMGMT].read;
    int pipe_w = pipes[TASKEXECUTOR].write;

    auto then = std::chrono::steady_clock::now();
    while (true) {
        bool receivedMessage = false;
        SingleStandardInformationRequest singleStandardInformationRequest;

        // TODO: Something that happens on pipe read from RELATIONMGMT


        receivedMessage |= receiveIPCClientRequest(server, singleStandardInformationRequest);
        
        // requestId_t requestId;
        // pid_t pid;
        // std::optional<NodeRequest> foo = server.receiveNodeRequest(requestId, pid, false);
        // if (foo.has_value()) {
        //     NodeRequest foo_val = foo.value();
        //     server.sendNodeResponse(NodeResponse{.alive=1}, pid);
        // }


        if (receivedMessage) {
            writeT<SingleStandardInformationRequest>(pipe_w, singleStandardInformationRequest);
            continue;
        }
        
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now-then);
        auto sleepTime = 1s - elapsed;
        if (sleepTime > 0s) {
            std::this_thread::sleep_for(sleepTime);
        }
        then = std::chrono::steady_clock::now();
    }
}

}