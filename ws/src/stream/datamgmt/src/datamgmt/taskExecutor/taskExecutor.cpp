#include <iostream>
#include <vector>
#include <bits/stdc++.h>
#include <unordered_set>
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
    CUSTOMVALUE,
};

struct CustomAttributesTask {
    std::string query;
    bool        continuous;
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
    // CustomAttribute
    CustomAttributesTask custom;
         

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


double getValueStandardQueryInfluxDB(Task task) {
    std::string request = influxDB::createPayloadGetSingleValue("STREAM", task.standardSingle.attribute, task.primaryKeys);
    std::string response = curl::push(request, curl::INFLUXDB_READ);

    return influxDB::extractValueFromCSV(response);
}

std::vector<std::string> split(std::string str, char delimiter) {
    std::stringstream ss(str);
    std::vector<std::string> res;
    std::string token;
    while (std::getline(ss, token, delimiter)) {
        res.push_back(token);
    }

    return res;
}

std::vector<std::string> getCustomResponseQueryInfluxDB(Task task) {
    std::string response = curl::push(task.custom.query, curl::INFLUXDB_READ);
    
    return split(response, '\n');
}

std::vector<primaryKey_t> parsePrimaryKeys(const std::string& jsonPayload) {
    std::vector<primaryKey_t> result;

    auto j = json::parse(jsonPayload);

    if (j.contains("results") && !j["results"].empty()) {
        auto& resultObj = j["results"][0];

        if (resultObj.contains("data") && !resultObj["data"].empty()) {
            auto& rowArray = resultObj["data"][0]["row"];

            if (!rowArray.empty() && rowArray[0].is_array()) {
                for (const auto& key : rowArray[0]) {
                    result.push_back(key.get<primaryKey_t>());
                }
            }
        }
    }

    return result;
}

std::vector<primaryKey_t> makeUnion(const std::vector<primaryKey_t>& vector1, const std::vector<primaryKey_t>& vector2) {
    std::unordered_set<primaryKey_t> resultSet(vector1.begin(), vector1.end());

    for (const primaryKey_t& val : vector2) {
        resultSet.insert(val);
    }

    return std::vector<primaryKey_t>(resultSet.begin(), resultSet.end());
}

std::vector<primaryKey_t> makeIntersection(const std::vector<primaryKey_t>& vector1, const std::vector<primaryKey_t>& vector2) {
    std::unordered_set<primaryKey_t> set1(vector1.begin(), vector1.end());
    std::unordered_set<primaryKey_t> resultSet;

    for (const primaryKey_t& val : vector2) {
        if (set1.find(val) != set1.end()) {
            resultSet.insert(val);
        }
    }

    return std::vector<primaryKey_t>(resultSet.begin(), resultSet.end());
}

std::vector<primaryKey_t> makeDifference(const std::vector<primaryKey_t>& vector1, const std::vector<primaryKey_t>& vector2) {
    std::unordered_set<primaryKey_t> toRemove(vector2.begin(), vector2.end());
    std::vector<primaryKey_t> result;

    for (primaryKey_t val : vector1) {
        if (toRemove.find(val) == toRemove.end()) {
            result.push_back(val);
        }
    }

    return result;
}

void taskExecutor(const IpcServer &server, std::map<Module_t, Pipe> pipes, std::atomic<bool> &running) {
    std::cout << "started taskExecutor" << std::endl;
    int pipe_r = pipes[TASKORCHESTRATOR].read;

    std::vector<Task> tasks;
    auto then = std::chrono::steady_clock::now();
    while (true) {
        {
            union_Tasks informationRequest;
            MsgType type;
            ssize_t ret = readT<union_Tasks>(pipe_r, informationRequest, &type);
            while (ret != -1) {
                switch (type)
                {
                case STANDARDSINGLE:
                    {
                        tasks.push_back(Task{
                        .pid            = informationRequest.standardSingle.pid,
                        .requestId      = informationRequest.standardSingle.requestID,
                        .type           = SINGLEVALUE,
                        .standardSingle = informationRequest.standardSingle.payload,
                        });
                        tasks.back().primaryKeys.push_back(informationRequest.standardSingle.payload.primaryKey);
                    }
                    break;
                case STANDARDAGGREGATED:
                    {
                        tasks.push_back(Task{
                            .pid                = informationRequest.standardAggregated.pid,
                            .requestId          = informationRequest.standardAggregated.requestID,
                            .type               = AGGREGATEDVALUE,
                            .standardAggregated = informationRequest.standardAggregated.payload,
                        });

                        // QUERY NEO4J with operation (DIFFERENCE)
                        std::string response = curl::push(tree::getPayloadForTree(
                            tasks.back().standardAggregated.primaryKey_RootTree1,
                            tasks.back().standardAggregated.tree1),
                            curl::NEO4J
                        );
                        std::vector<primaryKey_t> tree1 = parsePrimaryKeys(response);

                        response = curl::push(tree::getPayloadForTree(
                            tasks.back().standardAggregated.primaryKey_RootTree2,
                            tasks.back().standardAggregated.tree2),
                            curl::NEO4J
                        );
                        std::vector<primaryKey_t> tree2 = parsePrimaryKeys(response);
                        
                        std::vector<primaryKey_t> primaryKeys;
                        switch (informationRequest.standardAggregated.payload.binOperation)
                        {
                        case UNION:
                            primaryKeys = makeUnion(tree1, tree2);
                            break;
                        case INTERSECTION:
                            primaryKeys = makeIntersection(tree1, tree2);
                            break;
                        case DIFFERENCE:
                            primaryKeys = makeDifference(tree1, tree2);
                            break;
                        default:
                            std::cout << "unknown Operation" << std::endl;
                            break;
                        }
                        tasks.back().primaryKeys.insert(tasks.back().primaryKeys.end(), primaryKeys.begin(), primaryKeys.end());
                    }
                    break;
                case CUSTOM:
                    {
                        tasks.push_back(Task{
                            .pid        = informationRequest.custom.pid,
                            .requestId  = informationRequest.custom.requestID,
                            .type       = CUSTOMVALUE,
                            .custom     = CustomAttributesTask{
                                .continuous = informationRequest.custom.payload.continuous,
                            }
                        });
                        {
                            std::vector<std::string> queryLines = util::parseStringArray(informationRequest.custom.payload.query);
                            tasks.back().custom.query = std::accumulate(queryLines.begin(), queryLines.end(), std::string(" "));
                        }
                    }
                    break;
                default:
                    break;
                }

                ret = readT<union_Tasks>(pipe_r, informationRequest, &type);
            };
        }

        for (Task task : tasks) {
            switch (task.type)
            {
            case SINGLEVALUE:
            {
                StandardSingleAttributesResponse payload {
                    .value = getValueStandardQueryInfluxDB(task)
                };
                server.sendStandardSingleAttributesResponse(payload, task.pid, false);
                break;
            }
            case AGGREGATEDVALUE:
            {
                StandardAggregatedAttributesResponse payload {
                    .value = getValueStandardQueryInfluxDB(task)
                };
                server.sendStandardAggregatedAttributesResponse(payload, task.pid, false);
                break;
            }
            case CUSTOMVALUE:
            {
                CustomAttributesResponse payload;
                util::parseStringArray(payload.queryResponse, getCustomResponseQueryInfluxDB(task));
                server.sendCustomAttributesResponse(payload, task.pid, false);
                break;
            }
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