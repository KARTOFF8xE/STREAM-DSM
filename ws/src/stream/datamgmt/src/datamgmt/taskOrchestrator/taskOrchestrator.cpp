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
#include "datamgmt/common.hpp"
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

void addSingleAttributeTask(const IpcServer &server, SingleAttributeInformationRequest singleAttributeInformationRequest, Tasks &tasks) {
    std::lock_guard<std::mutex> lock(tasks.mutex);
    std::string memAddress = sharedMem::composeShmName(singleAttributeInformationRequest.pid, singleAttributeInformationRequest.requestID);

    SingleAttributesResponse resp { .requestID = singleAttributeInformationRequest.requestID };
    util::parseString(resp.memAddress, memAddress);

    if (singleAttributeInformationRequest.payload.direction == Direction::NONE) {
        tasks.vec.push_back(Task{
            .pid            = singleAttributeInformationRequest.pid,
            .requestId      = singleAttributeInformationRequest.requestID,
            .type           = SINGLEATTRIBUTE,
            .task           = singleAttributeInformationRequest.payload,
            .channel        = std::make_unique<sharedMem::SHMChannel<sharedMem::Response>>(resp.memAddress, true)
        });
    } else if (singleAttributeInformationRequest.payload.direction == Direction::EDGEINCOMING) {
        std::string payload = node::getAdjacentIncomingEdges(singleAttributeInformationRequest.payload.primaryKey);
        std::string response = curl::push(payload, curl::NEO4J);

        std::vector<std::string> primaryKeys;
        nlohmann::json j = nlohmann::json::parse(response);
        if (j.contains("results") && j["results"].is_array() && !j["results"].empty() &&
            j["results"][0].contains("data") && j["results"][0]["data"].is_array() && !j["results"][0]["data"].empty() &&
            j["results"][0]["data"][0].contains("row") && j["results"][0]["data"][0]["row"].is_array())
        {
            for (const auto& inner_array : j["results"][0]["data"][0]["row"]) {
                for (const auto& key : inner_array) {
                    primaryKeys.push_back(key.get<std::string>());
                }
            }
        }

        RelationTask relationTask {
            .attribute      = singleAttributeInformationRequest.payload.attribute,
            .direction      = singleAttributeInformationRequest.payload.direction,
            .continuous     = singleAttributeInformationRequest.payload.continuous
        };
        tasks.vec.push_back(Task{
            .pid            = singleAttributeInformationRequest.pid,
            .requestId      = singleAttributeInformationRequest.requestID,
            .type           = RELATION,
            .primaryKeys    = primaryKeys,
            .task           = relationTask,
            .channel        = std::make_unique<sharedMem::SHMChannel<sharedMem::Response>>(resp.memAddress, true)
        });
    } else if (singleAttributeInformationRequest.payload.direction == Direction::EDGEOUTGOING) {
        std::string payload = node::getAdjacentOutgoingEdges(singleAttributeInformationRequest.payload.primaryKey);
        std::string response = curl::push(payload, curl::NEO4J);

        std::vector<std::string> primaryKeys;
        nlohmann::json j = nlohmann::json::parse(response);
        if (j.contains("results") && j["results"].is_array() && !j["results"].empty() &&
            j["results"][0].contains("data") && j["results"][0]["data"].is_array() && !j["results"][0]["data"].empty() &&
            j["results"][0]["data"][0].contains("row") && j["results"][0]["data"][0]["row"].is_array())
        {
            for (const auto& inner_array : j["results"][0]["data"][0]["row"]) {
                for (const auto& key : inner_array) {
                    primaryKeys.push_back(key.get<std::string>());
                }
            }
        }

        RelationTask relationTask {
            .attribute      = singleAttributeInformationRequest.payload.attribute,
            .direction      = singleAttributeInformationRequest.payload.direction,
            .continuous     = singleAttributeInformationRequest.payload.continuous
        };
        tasks.vec.push_back(Task{
            .pid            = singleAttributeInformationRequest.pid,
            .requestId      = singleAttributeInformationRequest.requestID,
            .type           = RELATION,
            .primaryKeys    = primaryKeys,
            .task           = relationTask,
            .channel        = std::make_unique<sharedMem::SHMChannel<sharedMem::Response>>(resp.memAddress, true)
        });
    }

    tasks.vec.back().primaryKeys.push_back(singleAttributeInformationRequest.payload.primaryKey);
    server.sendSingleAttributesResponse(resp, singleAttributeInformationRequest.pid, false);
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

std::vector<std::string> makeUnion(const std::vector<std::string>& vector1, const std::vector<std::string>& vector2) {
    std::unordered_set<std::string> resultSet(vector1.begin(), vector1.end());

    for (const std::string& val : vector2) {
        resultSet.insert(val);
    }

    return std::vector<std::string>(resultSet.begin(), resultSet.end());
}

std::vector<std::string> makeIntersection(const std::vector<std::string>& vector1, const std::vector<std::string>& vector2) {
    std::unordered_set<std::string> set1(vector1.begin(), vector1.end());
    std::unordered_set<std::string> resultSet;

    for (const std::string& val : vector2) {
        if (set1.find(val) != set1.end()) {
            resultSet.insert(val);
        }
    }

    return std::vector<std::string>(resultSet.begin(), resultSet.end());
}

std::vector<std::string> makeDifference(const std::vector<std::string>& vector1, const std::vector<std::string>& vector2) {
    std::unordered_set<std::string> toRemove(vector2.begin(), vector2.end());
    std::vector<std::string> result;

    for (std::string val : vector1) {
        if (toRemove.find(val) == toRemove.end()) {
            result.push_back(val);
        }
    }

    return result;
}

std::vector<std::string> parsePrimaryKeys(const std::string& jsonPayload) {
    std::vector<std::string> result;

    auto j = json::parse(jsonPayload);

    if (j.contains("results") && !j["results"].empty()) {
        auto& resultObj = j["results"][0];

        if (resultObj.contains("data") && !resultObj["data"].empty()) {
            auto& rowArray = resultObj["data"][0]["row"];

            if (!rowArray.empty() && rowArray[0].is_array()) {
                for (const auto& key : rowArray[0]) {
                    result.push_back(key.get<std::string>());
                }
            }
        }
    }

    return result;
}

void addAggregatedAttributeTask(const IpcServer &server, AggregatedAttributeInformationRequest aggregatedAttributeInformationRequest, Tasks &tasks) {
    std::lock_guard<std::mutex> lock(tasks.mutex);
    std::string memAddress = sharedMem::composeShmName(aggregatedAttributeInformationRequest.pid, aggregatedAttributeInformationRequest.requestID);

    AggregatedAttributesResponse resp { .requestID = aggregatedAttributeInformationRequest.requestID };
    util::parseString(resp.memAddress, memAddress);

    tasks.vec.push_back(Task {
        .pid        = aggregatedAttributeInformationRequest.pid,
        .requestId  = aggregatedAttributeInformationRequest.requestID,
        .type       = AGGREGATEDATTRIBUTE,
        .task       = aggregatedAttributeInformationRequest.payload,
        .channel    = std::make_unique<sharedMem::SHMChannel<sharedMem::Response>>(resp.memAddress, true)
    });
    std::string response = curl::push(tree::getPayloadForTree(
        std::get<AggregatedAttributesRequest>(tasks.vec.back().task).rootedTree1.primaryKey,
        std::get<AggregatedAttributesRequest>(tasks.vec.back().task).rootedTree1.tree),
        curl::NEO4J
    );
    std::vector<std::string> tree1 = parsePrimaryKeys(response);

    response = curl::push(tree::getPayloadForTree(
        std::get<AggregatedAttributesRequest>(tasks.vec.back().task).rootedTree2.primaryKey,
        std::get<AggregatedAttributesRequest>(tasks.vec.back().task).rootedTree2.tree),
        curl::NEO4J
    );
    std::vector<std::string> tree2 = parsePrimaryKeys(response);

    std::vector<std::string> primaryKeys;
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
    server.sendAggregatedAttributesResponse(resp, aggregatedAttributeInformationRequest.pid, false);
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

void addCustomAttributeTask(const IpcServer &server, CustomAttributeInformationRequest customAttributeInformationRequest, Tasks &tasks) {
    std::lock_guard<std::mutex> lock(tasks.mutex);
    std::string memAddress = sharedMem::composeShmName(customAttributeInformationRequest.pid, customAttributeInformationRequest.requestID);

    CustomAttributesResponse resp { .requestID = customAttributeInformationRequest.requestID };
    util::parseString(resp.memAddress, memAddress);

    tasks.vec.push_back(Task{
        .pid        = customAttributeInformationRequest.pid,
        .requestId  = customAttributeInformationRequest.requestID,
        .type       = CUSTOMATTRIBUTE,
        .task       = CustomAttributesTask{.continuous = customAttributeInformationRequest.payload.continuous},
        .channel    = std::make_unique<sharedMem::SHMChannel<sharedMem::Response>>(resp.memAddress, true)
    });
    {
        std::vector<std::string> queryLines = util::parseStringArray(customAttributeInformationRequest.payload.query);
        std::get<CustomAttributesTask>(tasks.vec.back().task).query =
            std::accumulate(queryLines.begin(),
            queryLines.end(),
            std::string(""));
    }

    server.sendCustomAttributesResponse(resp, customAttributeInformationRequest.pid, false);
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

void addAggregatedMemberTask(const IpcServer &server, AggregatedMemberInformationRequest aggregatedMemberInformationRequest, Tasks &tasks) {
    std::lock_guard<std::mutex> lock(tasks.mutex);
    std::string memAddress = sharedMem::composeShmName(aggregatedMemberInformationRequest.pid, aggregatedMemberInformationRequest.requestID);

    AggregatedMemberResponse resp { .requestID = aggregatedMemberInformationRequest.requestID };
    util::parseString(resp.memAddress, memAddress);

    tasks.vec.push_back(Task{
        .pid        = aggregatedMemberInformationRequest.pid,
        .requestId  = aggregatedMemberInformationRequest.requestID,
        .type       = AGGREGATEDMEMBER,
        .task       = aggregatedMemberInformationRequest.payload,
        .channel    = std::make_unique<sharedMem::SHMChannel<sharedMem::Response>>(resp.memAddress, true)
    });
    std::string response = curl::push(tree::getPayloadForTree(
        std::get<AggregatedMemberRequest>(tasks.vec.back().task).rootedTree1.primaryKey,
        std::get<AggregatedMemberRequest>(tasks.vec.back().task).rootedTree1.tree),
        curl::NEO4J
    );
    std::vector<std::string> tree1 = parsePrimaryKeys(response);

    response = curl::push(tree::getPayloadForTree(
        std::get<AggregatedMemberRequest>(tasks.vec.back().task).rootedTree2.primaryKey,
        std::get<AggregatedMemberRequest>(tasks.vec.back().task).rootedTree2.tree),
        curl::NEO4J
    );
    std::vector<std::string> tree2 = parsePrimaryKeys(response);

    std::vector<std::string> primaryKeys;
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
    server.sendAggregatedMemberResponse(resp, aggregatedMemberInformationRequest.pid, false);
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

void addCustomMemberTask(const IpcServer &server,CustomMemberInformationRequest customMemberInformationRequest, Tasks &tasks) {
    std::lock_guard<std::mutex> lock(tasks.mutex);
    std::string memAddress = sharedMem::composeShmName(customMemberInformationRequest.pid, customMemberInformationRequest.requestID);

    CustomMemberResponse resp { .requestID = customMemberInformationRequest.requestID };
    util::parseString(resp.memAddress, memAddress);

    tasks.vec.push_back(Task{
        .pid        = customMemberInformationRequest.pid,
        .requestId  = customMemberInformationRequest.requestID,
        .type       = CUSTOMMEMBER,
        .task       = CustomMemberTask {.continuous = customMemberInformationRequest.payload.continuous},
        .channel    = std::make_unique<sharedMem::SHMChannel<sharedMem::Response>>(resp.memAddress, true)
    });
    {
        std::vector<std::string> queryLines = util::parseStringArray(customMemberInformationRequest.payload.query);
        std::get<CustomMemberTask>(tasks.vec.back().task).query =
            std::accumulate(queryLines.begin(),
            queryLines.end(),
            std::string(""));
    }
    server.sendCustomMemberResponse(resp, customMemberInformationRequest.pid, false);
}


void taskOrchestrator(const IpcServer &server, std::map<Module_t, pipe_ns::Pipe> pipes, std::atomic<bool> &running, Tasks &tasks) {
    std::cout << "started taskOrchestrator" << std::endl;
    int pipe_r = pipes[RELATIONMGMT].read;

    auto then = std::chrono::steady_clock::now();
    while (gsRunning) {
        bool receivedMessage = false;
        SingleAttributeInformationRequest       singleAttributeInformationRequest;
        AggregatedAttributeInformationRequest   aggregatedAttributeInformationRequest;
        CustomAttributeInformationRequest       customAttributeInformationRequest;
        AggregatedMemberInformationRequest      aggregatedMemberInformationRequest;
        CustomMemberInformationRequest          customMemberInformationRequest;


        // TODO: Something that happens on pipe read from RELATIONMGMT
        
        if (receiveIPCClientStandardSingleAttributeRequest(server, singleAttributeInformationRequest)) {
            addSingleAttributeTask(server, singleAttributeInformationRequest, tasks);
            continue;
        }
        if (receiveIPCClientStandardAggregatedAttributeRequest(server, aggregatedAttributeInformationRequest)) {
            addAggregatedAttributeTask(server, aggregatedAttributeInformationRequest, tasks);
            continue;
        }
        if (receiveIPCClientCustomAttributeRequest(server, customAttributeInformationRequest)) {
            addCustomAttributeTask(server, customAttributeInformationRequest, tasks);
            continue;
        }
        if (receiveIPCClientAggregatedMemberRequest(server, aggregatedMemberInformationRequest)) {
            addAggregatedMemberTask(server, aggregatedMemberInformationRequest, tasks);
            continue;
        }
        if (receiveIPCClientCustomMemberRequest(server, customMemberInformationRequest)) {
            addCustomMemberTask(server, customMemberInformationRequest, tasks);
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

    std::cout << "finalized taskOrchestrator" << std::endl;
}

}