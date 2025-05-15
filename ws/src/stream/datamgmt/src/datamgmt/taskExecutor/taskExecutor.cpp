#include <iostream>
#include <vector>
#include <nlohmann/json.hpp>
#include <chrono>
using namespace std::chrono_literals;

#include <ipc/ipc-client.hpp>
#include <ipc/util.hpp>
#include <curl/myCurl.hpp>
#include <neo4j/tree/tree.hpp>
#include <influxdb/influxdb.hpp>

#include "datamgmt/taskExecutor/taskExecutor.hpp"
#include "pipe/pipe.hpp"
#include "ipc/common.hpp"

using json = nlohmann::json;


enum RequestType {
    SINGLEVALUE,
    AGGREGATEDVALUE,
};

struct Task {
    pid_t                       pid;
    requestId_t                 requestId;
    RequestType                 type;
    std::vector<primaryKey_t>   primaryKeys;
    // SingleAttribute
    StandardSingleAttributesRequest standardSingle;
    // AggregatedAttribute
    StandardAggregatedAttributesRequest standardAggregated;

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
    std::string request = influxDB::createPayloadGetSingleValue("STREAM", task.standardSingle.attribute, task.primaryKeys);

    std::string response = curl::push(request, curl::INFLUXDB_READ);

    StandardSingleAttributesResponse payload {
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
        {
            SingleStandardInformationRequest singleStandardInformationRequest;
            ssize_t ret = readT<SingleStandardInformationRequest>(pipe_r, singleStandardInformationRequest);
            while (ret != -1) {
                if (singleStandardInformationRequest.payload.continuous = true) {
                    tasks.push_back(Task{
                        .pid            = singleStandardInformationRequest.pid,
                        .requestId      = singleStandardInformationRequest.requestID,
                        .type           = SINGLEVALUE,
                        .standardSingle = singleStandardInformationRequest.payload,
                    });
                    tasks.back().primaryKeys.push_back(singleStandardInformationRequest.payload.primaryKey);
                }

                ret = readT<SingleStandardInformationRequest>(pipe_r, singleStandardInformationRequest);
            };
        }
        {
            AggregatedStandardInformationRequest aggregatedStandardInformationRequest;
            ssize_t ret = readT<AggregatedStandardInformationRequest>(pipe_r, aggregatedStandardInformationRequest);
            while (ret != -1) {
                if (aggregatedStandardInformationRequest.payload.continuous = true) {
                    tasks.push_back(Task{
                        .pid                = aggregatedStandardInformationRequest.pid,
                        .requestId          = aggregatedStandardInformationRequest.requestID,
                        .type               = AGGREGATEDVALUE,
                        .standardAggregated = aggregatedStandardInformationRequest.payload,
                    });

                    // QUERY NEO4J with operation (DIFFERENCE)
                    std::vector<primaryKey_t> tree1;
                    std::vector<primaryKey_t> tree2;

                    std::string response = curl::push(tree::getPayloadForTree(
                            tasks.back().standardAggregated.primaryKey_RootTree1,
                            tasks.back().standardAggregated.tree1),
                        curl::NEO4J
                    );

                    std::cout << "IIIIIIIIIIII: " << response << std::endl;
                    // STORE in Vector
                }

                ret = readT<AggregatedStandardInformationRequest>(pipe_r, aggregatedStandardInformationRequest);
            };
        }

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