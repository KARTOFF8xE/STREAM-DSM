#include <iostream>
#include <vector>
#include <variant>
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
    AGGREGATEDMEMBERS,
    CUSTOMMEMBERS
};

struct CustomAttributesTask {
    std::string query;
    bool        continuous;
};

struct CustomMemberTask {
    std::string query;
    bool        continuous;
};

// union union_Tasks {
//     StandardSingleAttributesRequest     standardSingle;
//     StandardAggregatedAttributesRequest standardAggregated;
//     CustomAttributesTask                custom;
//     AggregatedMemberRequest             aggregatedMember;
//     CustomMemberTask                    customMember;


//     union_Tasks() {}
//     ~union_Tasks() {}
// };

using TaskVariant = std::variant<
    StandardSingleAttributesRequest,
    StandardAggregatedAttributesRequest,
    CustomAttributesTask,
    AggregatedMemberRequest,
    CustomMemberTask
>;

struct Task {
    pid_t                       pid;
    requestId_t                 requestId;
    RequestType                 type;
    std::vector<primaryKey_t>   primaryKeys;

    TaskVariant                 task;
         

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
    std::string request;
    if (std::holds_alternative<StandardSingleAttributesRequest>(task.task)) {
        request =
            influxDB::createPayloadGetSingleValue(
                "STREAM",
                std::get<StandardSingleAttributesRequest>(task.task).attribute,
                task.primaryKeys);
    }
    if (std::holds_alternative<StandardAggregatedAttributesRequest>(task.task)) {
        request =
            influxDB::createPayloadGetSingleValue(
                "STREAM",
                std::get<StandardAggregatedAttributesRequest>(task.task).attribute,
                task.primaryKeys);
    }

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
    std::string response =
        curl::push(
            std::get<CustomAttributesTask>(task.task).query,
            curl::INFLUXDB_READ);
    
    return split(response, '\n');
}

std::vector<std::string> getCustomResponseQueryNeo4J(Task task) {
    std::string query = R"(
          {
              "statements":
                  [
                      { "statement": ")";
    query += std::get<CustomMemberTask>(task.task).query + R"( "
                      }
                  ]
          })";
    std::string response = curl::push(query, curl::NEO4J);

    std::vector<std::string> responseSplitted;

    for (std::size_t i = 0; i < response.length(); i += MAX_STRING_SIZE-1) {
        responseSplitted.push_back(response.substr(i, MAX_STRING_SIZE-1));
    }
    
    return responseSplitted;
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

void taskExecutor(const IpcServer &server, std::map<Module_t, pipe_ns::Pipe> pipes, std::atomic<bool> &running) {
    std::cout << "started taskExecutor" << std::endl;
    int pipe_r = pipes[TASKORCHESTRATOR].read;

    std::vector<Task> tasks;
    auto then = std::chrono::steady_clock::now();
    while (true) {
        {
            union_Requests informationRequest;
            pipe_ns::MsgType type;
            ssize_t ret = pipe_ns::readT<union_Requests>(pipe_r, informationRequest, &type);
            while (ret != -1) {
                switch (type)
                {
                case pipe_ns::STANDARDSINGLEVALUE:
                    {
                        tasks.push_back(Task{
                        .pid            = informationRequest.standardSingle.pid,
                        .requestId      = informationRequest.standardSingle.requestID,
                        .type           = SINGLEVALUE,
                        .task           = informationRequest.standardSingle.payload,
                        });
                        tasks.back().primaryKeys.push_back(informationRequest.standardSingle.payload.primaryKey);
                    }
                    break;
                case pipe_ns::STANDARDAGGREGATEDVALUE:
                    {
                        tasks.push_back(Task{
                            .pid                = informationRequest.standardAggregated.pid,
                            .requestId          = informationRequest.standardAggregated.requestID,
                            .type               = AGGREGATEDVALUE,
                            .task               = informationRequest.standardAggregated.payload,
                        });
                        std::string response = curl::push(tree::getPayloadForTree(
                            std::get<StandardAggregatedAttributesRequest>(tasks.back().task).primaryKey_RootTree1,
                            std::get<StandardAggregatedAttributesRequest>(tasks.back().task).tree1),
                            curl::NEO4J
                        );
                        std::vector<primaryKey_t> tree1 = parsePrimaryKeys(response);

                        response = curl::push(tree::getPayloadForTree(
                            std::get<StandardAggregatedAttributesRequest>(tasks.back().task).primaryKey_RootTree2,
                            std::get<StandardAggregatedAttributesRequest>(tasks.back().task).tree2),
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
                case pipe_ns::CUSTOMVALUE:
                    {
                        tasks.push_back(Task{
                            .pid        = informationRequest.custom.pid,
                            .requestId  = informationRequest.custom.requestID,
                            .type       = CUSTOMVALUE,
                            .task       = CustomAttributesTask{.continuous = informationRequest.custom.payload.continuous}
                        });
                        {
                            std::vector<std::string> queryLines = util::parseStringArray(informationRequest.custom.payload.query);
                            std::get<CustomAttributesTask>(tasks.back().task).query =
                                std::accumulate(queryLines.begin(),
                                queryLines.end(),
                                std::string(""));
                        }
                    }
                    break;
                case pipe_ns::AGGREGATEDMEMBER:
                    {
                        tasks.push_back(Task{
                            .pid                = informationRequest.standardMember.pid,
                            .requestId          = informationRequest.standardMember.requestID,
                            .type               = AGGREGATEDMEMBERS,
                            .task               = informationRequest.standardMember.payload
                        });
                        std::string response = curl::push(tree::getPayloadForTree(
                            std::get<AggregatedMemberRequest>(tasks.back().task).primaryKey_RootTree1,
                            std::get<AggregatedMemberRequest>(tasks.back().task).tree1),
                            curl::NEO4J
                        );
                        std::vector<primaryKey_t> tree1 = parsePrimaryKeys(response);


                        response = curl::push(tree::getPayloadForTree(
                            std::get<AggregatedMemberRequest>(tasks.back().task).primaryKey_RootTree2,
                            std::get<AggregatedMemberRequest>(tasks.back().task).tree2),
                            curl::NEO4J
                        );
                        std::vector<primaryKey_t> tree2 = parsePrimaryKeys(response);

                        std::vector<primaryKey_t> primaryKeys;
                        switch (informationRequest.standardMember.payload.binOperation)
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
                case pipe_ns::CUSTOMMEMBER:
                    {
                        tasks.push_back(Task{
                            .pid            = informationRequest.customMember.pid,
                            .requestId      = informationRequest.customMember.requestID,
                            .type           = CUSTOMMEMBERS,
                            .task           = CustomMemberTask {.continuous = informationRequest.customMember.payload.continuous}
                        });
                        {
                            std::vector<std::string> queryLines = util::parseStringArray(informationRequest.custom.payload.query);
                            std::get<CustomMemberTask>(tasks.back().task).query =
                                std::accumulate(queryLines.begin(),
                                queryLines.end(),
                                std::string(""));
                        }
                    }
                    break;
                default:
                    break;
                }

                ret = pipe_ns::readT<union_Requests>(pipe_r, informationRequest, &type);
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

                if (!std::get<StandardSingleAttributesRequest>(task.task).continuous) {
                    tasks.erase(find(tasks.begin(), tasks.end(), task));
                }
                break;
            }
            case AGGREGATEDVALUE:
            {
                StandardAggregatedAttributesResponse payload {
                    .value = getValueStandardQueryInfluxDB(task)
                };
                server.sendStandardAggregatedAttributesResponse(payload, task.pid, false);

                if (!std::get<StandardAggregatedAttributesRequest>(task.task).continuous) {
                    tasks.erase(find(tasks.begin(), tasks.end(), task));
                }
                break;
            }
            case CUSTOMVALUE:
            {
                std::vector<std::string> response = getCustomResponseQueryInfluxDB(task);

                size_t counter = 0;
                for (auto line: response) {
                    CustomAttributesResponse payload {
                        .number = ++counter,
                        .total  = response.size(),
                    };
                    util::parseString(payload.line, line);

                    server.sendCustomAttributesResponse(payload, task.pid, false);
                }
                break;
            }
            case AGGREGATEDMEMBERS:
            {
                u_int16_t counter = 0;
                for (auto primKey: task.primaryKeys) {
                    AggregatedMemberResponse payload {
                        .number     = ++counter,
                        .total      = task.primaryKeys.size(),
                        .primaryKey = primKey                                        
                    };

                    server.sendAggregatedMemberResponse(payload, task.pid, false);
                }

                if (!std::get<AggregatedMemberRequest>(task.task).continuous) {
                    tasks.erase(find(tasks.begin(), tasks.end(), task));
                }

                break;
            }
            case CUSTOMMEMBERS:
            {
                std::vector<std::string> response = getCustomResponseQueryNeo4J(task);

                size_t counter = 0;
                for (auto line: response) {
                    CustomMemberResponse payload {
                        .number = ++counter,
                        .total  = response.size(),
                    };
                    util::parseString(payload.line, line);

                    server.sendCustomMemberResponse(payload, task.pid, false);
                }

                if (!std::get<CustomMemberTask>(task.task).continuous) {
                    tasks.erase(find(tasks.begin(), tasks.end(), task));
                }
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