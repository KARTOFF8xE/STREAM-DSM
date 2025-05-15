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
#include <influxdb/influxdb.hpp>

#include "datamgmt/taskExecutor/taskExecutor.hpp"
#include "pipe/pipe.hpp"
#include "ipc/common.hpp"

using json = nlohmann::json;


enum RequestType {
    SINGLEVALUE,
};

struct Task {
    pid_t           pid;
    requestId_t     requestId;
    primaryKey_t    primaryKey;
    Attribute       attribute;
    RequestType     type;

    bool operator==(const Task& other) const {
        return this->pid == other.pid;
    }
};


namespace taskExecutor {

bool receiveIPCClientRequest(const IpcServer &ipcServer, SingleStandardInformationRequest &receivedRequest) {
    std::optional<StandardSingleAttributesRequest> response =
        ipcServer.receiveStandardSingleAttributesRequest(receivedRequest.requestID, receivedRequest.pid, false);

    if (response.has_value()) {
        receivedRequest.payload = response.value();

        return true;
    }

    return false;
}


void querySingleValue(Task task, const IpcServer &server) {
    std::string request = influxDB::createPayloadGetSingleValue("STREAM", task.attribute, task.primaryKey);

    std::string response = curl::push(request, curl::INFLUXDB_READ);

    StandardSingleAttributesResponse payload {
        .primaryKey = task.primaryKey,
        .value = influxDB::extractValueFromCSV(response)
    };
    
    server.sendStandardSingleAttributesResponse(payload, task.pid, false);
}


void taskExecutor(const IpcServer &server, std::map<Module_t, Pipe> pipes, std::atomic<bool> &running) {
    std::cout << "started taskExecutor" << std::endl;
    int pipe_r = pipes[TASKORCHESTRATOR].read;

    std::vector<Task> tasks;
    auto then = std::chrono::steady_clock::now();
    while (true) {
        SingleStandardInformationRequest singleStandardInformationRequest;

        ssize_t ret = readT<SingleStandardInformationRequest>(pipe_r, singleStandardInformationRequest);
        while (ret != -1) {
            if (singleStandardInformationRequest.payload.continuous = true) {
                tasks.push_back(Task{
                    .pid        = singleStandardInformationRequest.pid,
                    .requestId  = singleStandardInformationRequest.requestID,
                    .primaryKey = singleStandardInformationRequest.payload.primaryKey,
                    .attribute  = singleStandardInformationRequest.payload.attribute,
                    .type       = SINGLEVALUE,
                });
            }

            ret = readT<SingleStandardInformationRequest>(pipe_r, singleStandardInformationRequest);
        };

        for (Task task : tasks) {
            switch (task.type)
            {
            case SINGLEVALUE:
                querySingleValue(task, server);
                break;
            default:
                break;
            }
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