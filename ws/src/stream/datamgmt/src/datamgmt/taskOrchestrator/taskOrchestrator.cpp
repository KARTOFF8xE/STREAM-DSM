#include <iostream>
#include <vector>
#include <unordered_set>
#include <nlohmann/json.hpp>
#include <chrono>
using namespace std::chrono_literals;

#include <ipc/ipc-client.hpp>
#include <ipc/util.hpp>
#include <curl/myCurl.hpp>
#include <neo4j/node/node.hpp>
#include <neo4j/topic/topic.hpp>
#include <neo4j/tree/tree.hpp>

#include "datamgmt/taskOrchestrator/taskOrchestrator.hpp"
#include "pipe/pipe.hpp"


using json = nlohmann::json;


namespace taskOrchestrator {

bool receiveIPCClientStandardSingleAttributeRequest(const IpcServer &ipcServer, SingleAttributeInformationRequest &receivedRequest) {
    std::optional<SingleAttributesRequest> response =
        ipcServer.receiveSingleAttributesRequest(receivedRequest.requestID, receivedRequest.pid, false);

    if (response.has_value()) {
        receivedRequest.payload = response.value();

        return true;
    }

    return false;
}

void addSingleAttributeTask(SingleAttributeInformationRequest singleAttributeInformationRequest, Tasks &tasks) {
    std::lock_guard<std::mutex> lock(tasks.mutex);
    tasks.vec.push_back(Task{
        .pid            = singleAttributeInformationRequest.pid,
        .requestId      = singleAttributeInformationRequest.requestID,
        .type           = SINGLEATTRIBUTE,
        .task           = singleAttributeInformationRequest.payload,
    });
    tasks.vec.back().primaryKeys.push_back(singleAttributeInformationRequest.payload.primaryKey);
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

void addAggregatedAttributeTask(AggregatedAttributeInformationRequest aggregatedAttributeInformationRequest, Tasks &tasks) {
    std::lock_guard<std::mutex> lock(tasks.mutex);
    tasks.vec.push_back(Task{
        .pid                = aggregatedAttributeInformationRequest.pid,
        .requestId          = aggregatedAttributeInformationRequest.requestID,
        .type               = AGGREGATEDATTRIBUTE,
        .task               = aggregatedAttributeInformationRequest.payload,
    });
    std::string response = curl::push(tree::getPayloadForTree(
        std::get<AggregatedAttributesRequest>(tasks.vec.back().task).primaryKey_RootTree1,
        std::get<AggregatedAttributesRequest>(tasks.vec.back().task).tree1),
        curl::NEO4J
    );
    std::vector<primaryKey_t> tree1 = parsePrimaryKeys(response);

    response = curl::push(tree::getPayloadForTree(
        std::get<AggregatedAttributesRequest>(tasks.vec.back().task).primaryKey_RootTree2,
        std::get<AggregatedAttributesRequest>(tasks.vec.back().task).tree2),
        curl::NEO4J
    );
    std::vector<primaryKey_t> tree2 = parsePrimaryKeys(response);

    std::vector<primaryKey_t> primaryKeys;
    switch (aggregatedAttributeInformationRequest.payload.binOperation)
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
    tasks.vec.back().primaryKeys.insert(tasks.vec.back().primaryKeys.end(), primaryKeys.begin(), primaryKeys.end());
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

void addCustomAttributeTask(CustomAttributeInformationRequest customAttributeInformationRequest, Tasks &tasks) {
    std::lock_guard<std::mutex> lock(tasks.mutex);
    tasks.vec.push_back(Task{
        .pid        = customAttributeInformationRequest.pid,
        .requestId  = customAttributeInformationRequest.requestID,
        .type       = CUSTOMATTRIBUTE,
        .task       = CustomAttributesTask{.continuous = customAttributeInformationRequest.payload.continuous}
    });
    {
        std::vector<std::string> queryLines = util::parseStringArray(customAttributeInformationRequest.payload.query);
        std::get<CustomAttributesTask>(tasks.vec.back().task).query =
            std::accumulate(queryLines.begin(),
            queryLines.end(),
            std::string(""));
    }
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

void addAggregatedMemberTask(AggregatedMemberInformationRequest aggregatedMemberInformationRequest, Tasks &tasks) {
    std::lock_guard<std::mutex> lock(tasks.mutex);
    tasks.vec.push_back(Task{
        .pid                = aggregatedMemberInformationRequest.pid,
        .requestId          = aggregatedMemberInformationRequest.requestID,
        .type               = AGGREGATEDMEMBER,
        .task               = aggregatedMemberInformationRequest.payload
    });
    std::string response = curl::push(tree::getPayloadForTree(
        std::get<AggregatedMemberRequest>(tasks.vec.back().task).primaryKey_RootTree1,
        std::get<AggregatedMemberRequest>(tasks.vec.back().task).tree1),
        curl::NEO4J
    );
    std::vector<primaryKey_t> tree1 = parsePrimaryKeys(response);

    response = curl::push(tree::getPayloadForTree(
        std::get<AggregatedMemberRequest>(tasks.vec.back().task).primaryKey_RootTree2,
        std::get<AggregatedMemberRequest>(tasks.vec.back().task).tree2),
        curl::NEO4J
    );
    std::vector<primaryKey_t> tree2 = parsePrimaryKeys(response);

    std::vector<primaryKey_t> primaryKeys;
    switch (aggregatedMemberInformationRequest.payload.binOperation)
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
    tasks.vec.back().primaryKeys.insert(tasks.vec.back().primaryKeys.end(), primaryKeys.begin(), primaryKeys.end());
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

void addCustomMemberTask(CustomMemberInformationRequest customMemberInformationRequest, Tasks &tasks) {
    std::lock_guard<std::mutex> lock(tasks.mutex);
    tasks.vec.push_back(Task{
        .pid            = customMemberInformationRequest.pid,
        .requestId      = customMemberInformationRequest.requestID,
        .type           = CUSTOMMEMBER,
        .task           = CustomMemberTask {.continuous = customMemberInformationRequest.payload.continuous}
    });
    {
        std::vector<std::string> queryLines = util::parseStringArray(customMemberInformationRequest.payload.query);
        std::get<CustomMemberTask>(tasks.vec.back().task).query =
            std::accumulate(queryLines.begin(),
            queryLines.end(),
            std::string(""));
    }
}


void taskOrchestrator(const IpcServer &server, std::map<Module_t, pipe_ns::Pipe> pipes, std::atomic<bool> &running, Tasks &tasks) {
    std::cout << "started taskOrchestrator" << std::endl;
    int pipe_r = pipes[RELATIONMGMT].read;

    auto then = std::chrono::steady_clock::now();
    while (true) {
        bool receivedMessage = false;
        SingleAttributeInformationRequest       singleAttributeInformationRequest;
        AggregatedAttributeInformationRequest   aggregatedAttributeInformationRequest;
        CustomAttributeInformationRequest       customAttributeInformationRequest;
        AggregatedMemberInformationRequest      aggregatedMemberInformationRequest;
        CustomMemberInformationRequest          customMemberInformationRequest;


        // TODO: Something that happens on pipe read from RELATIONMGMT
        
        if (receiveIPCClientStandardSingleAttributeRequest(server, singleAttributeInformationRequest)) {
            addSingleAttributeTask(singleAttributeInformationRequest, tasks);
            continue;
        }
        if (receiveIPCClientStandardAggregatedAttributeRequest(server, aggregatedAttributeInformationRequest)) {
            addAggregatedAttributeTask(aggregatedAttributeInformationRequest, tasks);
            continue;
        }
        if (receiveIPCClientCustomAttributeRequest(server, customAttributeInformationRequest)) {
            addCustomAttributeTask(customAttributeInformationRequest, tasks);
            continue;
        }
        if (receiveIPCClientAggregatedMemberRequest(server, aggregatedMemberInformationRequest)) {
            addAggregatedMemberTask(aggregatedMemberInformationRequest, tasks);
            continue;
        }
        if (receiveIPCClientCustomMemberRequest(server, customMemberInformationRequest)) {
            addCustomMemberTask(customMemberInformationRequest, tasks);
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