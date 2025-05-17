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

// TODO: some day prettify using macros
bool receiveIPCClientStandardSingleValueRequest(const IpcServer &ipcServer, SingleStandardInformationRequest &receivedRequest) {
    std::optional<StandardSingleAttributesRequest> response =
        ipcServer.receiveStandardSingleAttributesRequest(receivedRequest.requestID, receivedRequest.pid, false);

    if (response.has_value()) {
        receivedRequest.payload = response.value();

        return true;
    }

    return false;
}

bool receiveIPCClientStandardAggregatedValueRequest(const IpcServer &ipcServer, AggregatedStandardInformationRequest &receivedRequest) {
    std::optional<StandardAggregatedAttributesRequest> response =
        ipcServer.receiveStandardAggregatedAttributesRequest(receivedRequest.requestID, receivedRequest.pid, false);

    if (response.has_value()) {
        receivedRequest.payload = response.value();

        return true;
    }

    return false;
}

bool receiveIPCClientCustomValueRequest(const IpcServer &ipcServer, CustomInformationRequest &receivedRequest) {
    std::optional<CustomAttributesRequest> response =
        ipcServer.receiveCustomAttributesRequest(receivedRequest.requestID, receivedRequest.pid, false);

    if (response.has_value()) {
        receivedRequest.payload = response.value();

        return true;
    }

    return false;
}

bool receiveIPCClientAggregatedMemberRequest(const IpcServer &ipcServer, StandardInformationMemberRequest &receivedRequest) {
    std::optional<AggregatedMemberRequest> response =
        ipcServer.receiveAggregatedMemberRequest(receivedRequest.requestID, receivedRequest.pid, false);

    if (response.has_value()) {
        receivedRequest.payload = response.value();

        return true;
    }

    return false;
}

bool receiveIPCClientCustomMemberRequest(const IpcServer &ipcServer, CustomInformationMemberRequest &receivedRequest) {
    std::optional<CustomMemberRequest> response =
        ipcServer.receiveCustomMemberRequest(receivedRequest.requestID, receivedRequest.pid, false);

    if (response.has_value()) {
        receivedRequest.payload = response.value();

        return true;
    }

    return false;
}


void taskOrchestrator(const IpcServer &server, std::map<Module_t, pipe_ns::Pipe> pipes, std::atomic<bool> &running) {
    std::cout << "started taskOrchestrator" << std::endl;
    int pipe_r = pipes[RELATIONMGMT].read;
    int pipe_w = pipes[TASKEXECUTOR].write;

    auto then = std::chrono::steady_clock::now();
    while (true) {
        bool receivedMessage = false;
        SingleStandardInformationRequest        singleStandardInformationRequest;
        AggregatedStandardInformationRequest    aggregatedStandardInformationRequest;
        CustomInformationRequest                customInformationRequest;
        StandardInformationMemberRequest        standardInformationMemberRequest;
        CustomInformationMemberRequest          customInformationMemberRequest;


        // TODO: Something that happens on pipe read from RELATIONMGMT
        
        // TODO: some day prettify using macros
        if (receiveIPCClientStandardSingleValueRequest(server, singleStandardInformationRequest)) {
            pipe_ns::writeT<SingleStandardInformationRequest>(pipe_w, singleStandardInformationRequest, pipe_ns::STANDARDSINGLEVALUE);
            continue;
        }
        if (receiveIPCClientStandardAggregatedValueRequest(server, aggregatedStandardInformationRequest)) {
            pipe_ns::writeT<AggregatedStandardInformationRequest>(pipe_w, aggregatedStandardInformationRequest, pipe_ns::STANDARDAGGREGATEDVALUE);
            continue;
        }
        if (receiveIPCClientCustomValueRequest(server, customInformationRequest)) {
            pipe_ns::writeT<CustomInformationRequest>(pipe_w, customInformationRequest, pipe_ns::CUSTOMVALUE);
            continue;
        }
        if (receiveIPCClientAggregatedMemberRequest(server, standardInformationMemberRequest)) {
            pipe_ns::writeT<StandardInformationMemberRequest>(pipe_w, standardInformationMemberRequest, pipe_ns::AGGREGATEDMEMBER);
            continue;
        }
        if (receiveIPCClientCustomMemberRequest(server, customInformationMemberRequest)) {
            pipe_ns::writeT<CustomInformationMemberRequest>(pipe_w, customInformationMemberRequest, pipe_ns::CUSTOMMEMBER);
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