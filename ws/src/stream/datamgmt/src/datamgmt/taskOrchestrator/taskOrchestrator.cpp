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
bool receiveIPCClientStandardSingleAttributeRequest(const IpcServer &ipcServer, SingleAttributeInformationRequest &receivedRequest) {
    std::optional<SingleAttributesRequest> response =
        ipcServer.receiveSingleAttributesRequest(receivedRequest.requestID, receivedRequest.pid, false);

    if (response.has_value()) {
        receivedRequest.payload = response.value();

        return true;
    }

    return false;
}

bool receiveIPCClientStandardAggregatedAttributeRequest(const IpcServer &ipcServer, AggregatedAttributeInformationRequest &receivedRequest) {
    std::optional<AggregatedAttributesRequest> response =
        ipcServer.receiveAggregatedAttributesRequest(receivedRequest.requestID, receivedRequest.pid, false);

    if (response.has_value()) {
        receivedRequest.payload = response.value();

        return true;
    }

    return false;
}

bool receiveIPCClientCustomAttributeRequest(const IpcServer &ipcServer, CustomAttributeInformationRequest &receivedRequest) {
    std::optional<CustomAttributesRequest> response =
        ipcServer.receiveCustomAttributesRequest(receivedRequest.requestID, receivedRequest.pid, false);

    if (response.has_value()) {
        receivedRequest.payload = response.value();

        return true;
    }

    return false;
}

bool receiveIPCClientAggregatedMemberRequest(const IpcServer &ipcServer, AggregatedMemberInformationRequest &receivedRequest) {
    std::optional<AggregatedMemberRequest> response =
        ipcServer.receiveAggregatedMemberRequest(receivedRequest.requestID, receivedRequest.pid, false);

    if (response.has_value()) {
        receivedRequest.payload = response.value();

        return true;
    }

    return false;
}

bool receiveIPCClientCustomMemberRequest(const IpcServer &ipcServer, CustomMemberInformationRequest &receivedRequest) {
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
        SingleAttributeInformationRequest       singleAttributeInformationRequest;
        AggregatedAttributeInformationRequest   aggregatedAttributeInformationRequest;
        CustomAttributeInformationRequest       customAttributeInformationRequest;
        AggregatedMemberInformationRequest      aggregatedMemberInformationRequest;
        CustomMemberInformationRequest          customMemberInformationRequest;


        // TODO: Something that happens on pipe read from RELATIONMGMT
        
        // TODO: some day prettify using macros
        if (receiveIPCClientStandardSingleAttributeRequest(server, singleAttributeInformationRequest)) {
            pipe_ns::writeT<SingleAttributeInformationRequest>(pipe_w, singleAttributeInformationRequest, pipe_ns::SINGLEATTRIBUTE);
            continue;
        }
        if (receiveIPCClientStandardAggregatedAttributeRequest(server, aggregatedAttributeInformationRequest)) {
            pipe_ns::writeT<AggregatedAttributeInformationRequest>(pipe_w, aggregatedAttributeInformationRequest, pipe_ns::AGGREGATEDATTRIBUTE);
            continue;
        }
        if (receiveIPCClientCustomAttributeRequest(server, customAttributeInformationRequest)) {
            pipe_ns::writeT<CustomAttributeInformationRequest>(pipe_w, customAttributeInformationRequest, pipe_ns::CUSTOMATTRIBUTE);
            continue;
        }
        if (receiveIPCClientAggregatedMemberRequest(server, aggregatedMemberInformationRequest)) {
            pipe_ns::writeT<AggregatedMemberInformationRequest>(pipe_w, aggregatedMemberInformationRequest, pipe_ns::AGGREGATEDMEMBER);
            continue;
        }
        if (receiveIPCClientCustomMemberRequest(server, customMemberInformationRequest)) {
            pipe_ns::writeT<CustomMemberInformationRequest>(pipe_w, customMemberInformationRequest, pipe_ns::CUSTOMMEMBER);
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