#include <iostream>
#include <vector>
#include <nlohmann/json.hpp>
#include <chrono>
#include <thread>
using namespace std::chrono_literals;

#include <ipc/ipc-client.hpp>
#include <ipc/sharedMem.hpp>
#include <ipc/util.hpp>
#include <curl/myCurl.hpp>
#include <influxdb/influxdb.hpp>
#include <neo4j/node/node.hpp>
#include <neo4j/topic/topic.hpp>
#include <neo4j/publisher/publisher.hpp>
#include <neo4j/subscriber/subscriber.hpp>
#include <neo4j/service/service.hpp>
#include <neo4j/client/client.hpp>
#include <neo4j/actionservice/actionservice.hpp>
#include <neo4j/actionclient/actionclient.hpp>
#include <neo4j/timer/timer.hpp>

#include "datamgmt/nodeandtopicobserver/nodeandtopicobserver.hpp"
#include "datamgmt/relationmgmt/relationmgmt.hpp"
#include "datamgmt/datamgmt.hpp"
#include <datamgmt/utils.hpp>
#include "datamgmt/common.hpp"
#include "pipe/pipe.hpp"

using json = nlohmann::json;


void singleTimeNodeResponse(const IpcServer &server, RequestingClient &client, std::string primaryKey) {
    std::string response = curl::push(node::getPayloadRequestByPrimaryKey(primaryKey), curl::NEO4J);

    json payload = json::parse(response);
    json row = payload["results"][0]["data"][0]["row"];

    json requestedNode;
    std::vector<NodePublishersToUpdate>         pubs;
    std::vector<NodeSubscribersToUpdate>        subs;
    std::vector<NodeIsServerForUpdate>          isServerFor;
    std::vector<NodeIsClientOfUpdate>           isClientOf;
    std::vector<NodeIsActionServerForUpdate>    isActionServerFor;
    std::vector<NodeIsActionClientOfUpdate>     isActionClientOf;
    std::vector<NodeTimerToUpdate>              timers;
    for (const json & item: row) {
        if (item.contains("name")) {
            requestedNode = item;
            continue;
        }

        size_t counter = 0;
        while (counter < item.size() && item[counter].contains("relationship")) {
            if (item[counter]["relationship"] == "publishing") {
                for (const json & node: item[counter]["nodes"]) {
                    NodePublishersToUpdate tmp { .isUpdate = false};
                    util::parseString(tmp.primaryKey, primaryKey);
                    util::parseString(tmp.publishesTo, node["id"].get<std::string>());
                    util::parseString(tmp.edge, node["edge_id"].get<std::string>());
                    pubs.push_back(tmp);
                }
            }
            if (item[counter]["relationship"] == "subscribing") {
                for (const json & node: item[counter]["nodes"]) {
                    NodeSubscribersToUpdate tmp { .isUpdate = false };
                    util::parseString(tmp.primaryKey, primaryKey);
                    util::parseString(tmp.subscribesTo, node["id"].get<std::string>());
                    util::parseString(tmp.edge, node["edge_id"].get<std::string>());
                    subs.push_back(tmp);
                }
            }
            if (item[counter]["relationship"] == "sending") {       // concept of Services
                for (const json & node: item[counter]["nodes"]) {
                    if (!node["actionName"].is_null() || node["direction"] != "outgoing") { continue; }
                    if (node["mode"] == "responds") {               // Service for...
                        NodeIsServerForUpdate tmp { .isUpdate = false };
                        util::parseString(tmp.srvName, node["serviceName"].get<std::string>());
                        util::parseString(tmp.primaryKey, primaryKey);
                        util::parseString(tmp.clientNodeId, node["id"].get<std::string>());

                        isServerFor.push_back(tmp);
                    } else {                                        // Client of...
                        NodeIsClientOfUpdate tmp { .isUpdate = false };
                        util::parseString(tmp.srvName, node["serviceName"].get<std::string>());
                        util::parseString(tmp.primaryKey, primaryKey);
                        util::parseString(tmp.serverNodeId, node["id"].get<std::string>());

                        isClientOf.push_back(tmp);
                    }
                }
            }
            if (item[counter]["relationship"] == "sending") {       // concept of Actions
                for (const json & node: item[counter]["nodes"]) {
                    if (node["actionName"].is_null() || node["direction"] != "outgoing" || node["serviceName"] != "send_goal") { continue; }
                    if (node["mode"] == "responds") {          // ActionService for...
                        NodeIsActionServerForUpdate tmp { .isUpdate = false };
                        util::parseString(tmp.srvName, node["actionName"].get<std::string>());
                        util::parseString(tmp.primaryKey, primaryKey);
                        util::parseString(tmp.actionclientNodeId, node["id"].get<std::string>());

                        isActionServerFor.push_back(tmp);
                    } else {                                        // ActionClient of...
                        NodeIsActionClientOfUpdate tmp { .isUpdate = false };
                        util::parseString(tmp.srvName, node["actionName"].get<std::string>());
                        util::parseString(tmp.primaryKey, primaryKey);
                        util::parseString(tmp.actionserverNodeId, node["id"].get<std::string>());

                        isActionClientOf.push_back(tmp);
                    }
                }
            }
            if (item[counter]["relationship"] == "timer") {
                for (const json & node: item[counter]["nodes"]) {
                    if (node["direction"] == "incoming") {
                        NodeTimerToUpdate tmp {
                            .frequency  = node["frequency"],
                            .isUpdate   = false,
                        };
                        util::parseString(tmp.primaryKey, primaryKey);

                        timers.push_back(tmp);
                    }
                }
            }
            counter++;
        }
    }

    NodeResponse nodeResponse {
        .state              = requestedNode["state"],
        .stateChangeTime    = requestedNode["stateChangeTime"],
        .bootCount          = requestedNode["bootcounter"],
        .pid                = requestedNode["pid"],
        .nrOfInitialUpdates = pubs.size() + subs.size() +
            isClientOf.size() + isServerFor.size() +
            isActionClientOf.size() + isActionServerFor.size(),
    };
    util::parseString(nodeResponse.primaryKey, primaryKey);
    util::parseString(nodeResponse.name, requestedNode["name"].get<std::string>());
    server.sendNodeResponse(nodeResponse, client.pid);

    for (NodePublishersToUpdate item : pubs) {
        server.sendNodePublishersToUpdate(item, client.pid);
    }
    for (NodeSubscribersToUpdate item : subs) {
        server.sendNodeSubscribersToUpdate(item, client.pid);
    }
    for (NodeIsClientOfUpdate item : isClientOf) {
        server.sendNodeIsClientOfUpdate(item, client.pid);
    }
    for (NodeIsServerForUpdate item : isServerFor) {
        server.sendNodeIsServerForUpdate(item, client.pid);
    }
    for (NodeIsActionClientOfUpdate item : isActionClientOf) {
        server.sendNodeIsActionClientOfUpdate(item, client.pid);
    }
    for (NodeIsActionServerForUpdate item : isActionServerFor) {
        server.sendNodeIsActionServerForUpdate(item, client.pid);
    }
    for (NodeTimerToUpdate timer : timers) {
        server.sendNodeTimerToUpdate(timer, client.pid);
    }
}

void singleTimeTopicResponse(const IpcServer &server, RequestingClient &client, std::string primaryKey) {
    std::string response = curl::push(topic::getPayloadRequestByPrimaryKey(primaryKey), curl::NEO4J);

    json payload = json::parse(response);
    json row = payload["results"][0]["data"][0]["row"];

    std::vector<TopicPublishersUpdate>  pubs;
    std::vector<TopicSubscribersUpdate> subs;
    for (const auto item : row[0]["publishers"]) {
        TopicPublishersUpdate tmp { .isUpdate = false };
        util::parseString(tmp.primaryKey, primaryKey);
        std::cout << item << std::endl;
        if (!item["node_id"].is_null()) {
            std::cout << "pub" << std::endl;
            util::parseString(tmp.publisher, item["node_id"].get<std::string>());
            util::parseString(tmp.edge, item["edge_id"].get<std::string>());
            pubs.push_back(tmp);
        }
    }
    for (const auto item : row[0]["subscribers"]) {
        TopicSubscribersUpdate tmp { .isUpdate = false };
        util::parseString(tmp.primaryKey, primaryKey);
        std::cout << item << std::endl;
        if (!item["node_id"].is_null()) {
            std::cout << "sub" << std::endl;
            util::parseString(tmp.subscriber, item["node_id"].get<std::string>());
            util::parseString(tmp.edge, item["edge_id"].get<std::string>());
            subs.push_back(tmp);
        }
    }

    TopicResponse topicResponse { .nrOfInitialUpdates = pubs.size() + subs.size() };
    util::parseString(topicResponse.primaryKey, primaryKey);
    util::parseString(topicResponse.name, row[0]["name"].get<std::string>());

    server.sendTopicResponse(topicResponse, client.pid);

    for (TopicPublishersUpdate item : pubs) {
        server.sendTopicPublishersUpdate(item, client.pid);
    }
    for (TopicSubscribersUpdate item : subs) {
        server.sendTopicSubscribersUpdate(item, client.pid);
    }
}


std::vector<NodeIsClientOfUpdate> queryGraphDbForClient(std::string payload, std::string srvName) {
    std::vector<NodeIsClientOfUpdate> nodeIsClientOfUpdate;

    NodeIsClientOfUpdate nodeIsClientOfUpdateBlueprint;
    util::parseString(nodeIsClientOfUpdateBlueprint.srvName, srvName);

    std::string response = curl::push(payload, curl::NEO4J);
    
    json data = nlohmann::json::parse(response);
    json row = data["results"][0]["data"][0]["row"];
    if (!row.empty() && !row[0].empty() && !row[0][0]["node_id"].empty()) util::parseString(nodeIsClientOfUpdateBlueprint.primaryKey, row[0][0]["node_id"]);
    
    size_t counter = 0;
    while (!row.empty() &&
        !row[0].empty() &&
        !row[0][counter].empty() &&
        !row[0][counter]["server_id"].empty()
        ) {
            util::parseString(nodeIsClientOfUpdateBlueprint.serverNodeId, row[0][counter]["server_id"]);
            nodeIsClientOfUpdate.push_back(nodeIsClientOfUpdateBlueprint);
        counter++;
    }

    return nodeIsClientOfUpdate;
}

void handleIsClientToUpdate(sharedMem::TraceMessage msg, std::vector<RequestingClient> &clients, const IpcServer &server) {
    std::string payloadNeo4j = client::getPayload(msg.client.srvName, msg.client.nodeHandle);
    std::vector<NodeIsClientOfUpdate> nodeIsCliToUpdate = queryGraphDbForClient(payloadNeo4j, msg.service.name);

    for (RequestingClient &client: clients) {
        for (NodeIsClientOfUpdate item : nodeIsCliToUpdate) {
            if (client.primaryKey == item.primaryKey) {
                server.sendNodeIsClientOfUpdate(item, client.pid, false);
            }
            if (client.primaryKey == item.serverNodeId) {
                NodeIsClientOfUpdate msg { .isUpdate = true };
                util::parseString(msg.primaryKey, item.serverNodeId);
                util::parseString(msg.serverNodeId, item.primaryKey);
                util::parseString(msg.srvName, item.srvName);

                server.sendNodeIsClientOfUpdate(msg, client.pid, false);
            }
        }
    }
}

std::vector<NodeIsServerForUpdate> queryGraphDbForService(std::string payload, std::string srvName) {
    std::vector<NodeIsServerForUpdate> nodeIsServerForUpdate;

    NodeIsServerForUpdate nodeIsServerForUpdateBlueprint;
    util::parseString(nodeIsServerForUpdateBlueprint.srvName, srvName);
    
    std::string response = curl::push(payload, curl::NEO4J);
    
    json data = nlohmann::json::parse(response);
    json row = data["results"][0]["data"][0]["row"];

    if (!row.empty() && !row[0].empty() && !row[0][0]["node_id"].empty()) util::parseString(nodeIsServerForUpdateBlueprint.primaryKey, row[0][0]["node_id"]);

    size_t counter = 0;
    while (!row.empty() &&
        !row[0].empty() &&
        !row[0][counter].empty() &&
        !row[0][counter]["client_id"].empty()
        ) {
            util::parseString(nodeIsServerForUpdateBlueprint.clientNodeId, row[0][counter]["client_id"]);
            nodeIsServerForUpdate.push_back(nodeIsServerForUpdateBlueprint);
        counter++;
    }

    return nodeIsServerForUpdate;
}

void handleIsServerForUpdate(sharedMem::TraceMessage msg, std::vector<RequestingClient> &clients, const IpcServer &server) {
    std::string payloadNeo4j = service::getPayload(msg.service.name, msg.service.nodeHandle);
    std::vector<NodeIsServerForUpdate> nodeIsSrvForUpdate = queryGraphDbForService(payloadNeo4j, msg.service.name);
    for (RequestingClient &client: clients) {
        for (NodeIsServerForUpdate item : nodeIsSrvForUpdate) {
            if (client.primaryKey == item.primaryKey) {
                server.sendNodeIsServerForUpdate(item, client.pid, false);
            }
            if (client.primaryKey == item.clientNodeId) {
                NodeIsClientOfUpdate msg{ .isUpdate = true };
                util::parseString(msg.primaryKey, item.clientNodeId);
                util::parseString(msg.serverNodeId, item.primaryKey);
                util::parseString(msg.srvName, item.srvName);

                server.sendNodeIsClientOfUpdate(msg, client.pid, false);
            }
        }
    }
}

void handleActionClientToUpdate(sharedMem::TraceMessage msg, std::vector<RequestingClient> &clients, const IpcServer &server) {
    char serviceName[MAX_STRING_SIZE];
    char actionName[MAX_STRING_SIZE];
    strncpy(serviceName, msg.service.name, MAX_STRING_SIZE);
    strncpy(actionName, msg.service.name, MAX_STRING_SIZE);
    truncateAfterSubstring(serviceName, "/_action/");
    truncateAtSubstring(actionName, "/_action/");

    std::string payloadNeo4j = actionclient::getPayload(serviceName, msg.client.nodeHandle, actionName);
    std::vector<NodeIsClientOfUpdate> nodeIsCliToUpdate = queryGraphDbForClient(payloadNeo4j, msg.service.name);
    
    if (strcmp(serviceName, "send_goal")) {
        for (RequestingClient &client: clients) {
            for (NodeIsClientOfUpdate item : nodeIsCliToUpdate) {
                if (client.primaryKey == item.primaryKey) {
                    server.sendNodeIsClientOfUpdate(item, client.pid, false);
                }
                if (client.primaryKey == item.serverNodeId) {
                    NodeIsClientOfUpdate msg{ .isUpdate = true };
                    util::parseString(msg.primaryKey, item.serverNodeId);
                    util::parseString(msg.serverNodeId, item.primaryKey);
                    util::parseString(msg.srvName, item.srvName);

                    server.sendNodeIsClientOfUpdate(msg, client.pid, false);
                }
            }
        }
    }
}

void handleActionServerForUpdate(sharedMem::TraceMessage msg, std::vector<RequestingClient> &clients, const IpcServer &server) {
    char serviceName[MAX_STRING_SIZE];
    char actionName[MAX_STRING_SIZE];
    strncpy(serviceName, msg.service.name, MAX_STRING_SIZE);
    strncpy(actionName, msg.service.name, MAX_STRING_SIZE);
    truncateAfterSubstring(serviceName, "/_action/");
    truncateAtSubstring(actionName, "/_action/");

    std::string payloadNeo4j = actionservice::getPayload(serviceName, msg.service.nodeHandle, actionName);
    std::vector<NodeIsServerForUpdate> nodeIsSrvForUpdate = queryGraphDbForService(payloadNeo4j, msg.service.name);

    if (strcmp(serviceName, "send_goal")) {
        for (RequestingClient &client: clients) {
            for (NodeIsServerForUpdate item : nodeIsSrvForUpdate) {
                if (client.primaryKey == item.primaryKey) {
                    server.sendNodeIsServerForUpdate(item, client.pid, false);
                }
                if (client.primaryKey == item.clientNodeId) {
                    NodeIsClientOfUpdate msg{ .isUpdate = true };
                    util::parseString(msg.primaryKey, item.clientNodeId);
                    util::parseString(msg.serverNodeId, item.primaryKey);
                    util::parseString(msg.srvName, item.srvName);

                    server.sendNodeIsClientOfUpdate(msg, client.pid, false);
                }
            }
        }
    }
}

NodeSubscribersToUpdate queryGraphDbForSubscriber(std::string payload) {
    NodeSubscribersToUpdate nodeSubToUpdate;

    std::string response = curl::push(payload, curl::NEO4J);

    json data = nlohmann::json::parse(response);
    json row = data["results"][0]["data"][0]["row"];
    if (!row.empty() && !row[0]["node_id"].empty())   util::parseString(nodeSubToUpdate.primaryKey, row[0]["node_id"].get<std::string>());
    if (!row.empty() && !row[0]["topic_id"].empty())  util::parseString(nodeSubToUpdate.subscribesTo, row[0]["topic_id"].get<std::string>());
    if (!row.empty() && !row[0]["edge_id"].empty())   util::parseString(nodeSubToUpdate.edge, row[0]["edge_id"].get<std::string>());

    return nodeSubToUpdate;
}

void handleSubscribersUpdate(sharedMem::TraceMessage msg, std::vector<RequestingClient> &clients, const IpcServer &server, int pipeToRelationMgmt_w) {
    std::string payloadNeo4j = subscriber::getPayload(msg.subscriber.topicName, msg.subscriber.nodeHandle);
    NodeSubscribersToUpdate nodeSubToUpdate = queryGraphDbForSubscriber(payloadNeo4j);

    if (!strstr(msg.subscriber.topicName, "/_action/")) {
        for (RequestingClient &client : clients)
        {
            if (client.primaryKey == nodeSubToUpdate.primaryKey)
            {
                server.sendNodeSubscribersToUpdate(nodeSubToUpdate, client.pid, false);
            }
        }

        TopicSubscribersUpdate payloadTopic { .isUpdate = true };
        util::parseString(payloadTopic.primaryKey, nodeSubToUpdate.subscribesTo);
        util::parseString(payloadTopic.subscriber, nodeSubToUpdate.primaryKey);
        util::parseString(payloadTopic.edge, nodeSubToUpdate.edge);
        for (RequestingClient &client : clients)
        {
            if (client.primaryKey == payloadTopic.primaryKey)
            {
                server.sendTopicSubscribersUpdate(payloadTopic, client.pid, false);
            }
        }
    }

    pipe_ns::UnionResponse unionResp;
    std::memcpy(unionResp.topicResp.name, msg.subscriber.topicName, sizeof(msg.subscriber.topicName));
    pipe_ns::writeT<pipe_ns::UnionResponse>(pipeToRelationMgmt_w, unionResp, pipe_ns::MsgType::TOPICRESPONSE);
}

NodePublishersToUpdate queryGraphDbForPublisher(std::string payload) {
    NodePublishersToUpdate nodePubToUpdate;

    std::string response = curl::push(payload, curl::NEO4J);
    json data = nlohmann::json::parse(response);
    json row = data["results"][0]["data"][0]["row"];

    if (!row.empty() && !row[0]["node_id"].empty())     util::parseString(nodePubToUpdate.primaryKey,   row[0]["node_id"].get<std::string>());
    if (!row.empty() && !row[0]["topic_id"].empty())    util::parseString(nodePubToUpdate.publishesTo,  row[0]["topic_id"].get<std::string>());
    if (!row.empty() && !row[0]["edge_id"].empty())     util::parseString(nodePubToUpdate.edge,         row[0]["edge_id"].get<std::string>());
    std::vector<std::string> incomingEdges;
    if (!row.empty() && row[0].contains("incomingEdges") && row[0]["incomingEdges"].is_array()) {
        for (const auto& edge : row[0]["incomingEdges"]) {
            incomingEdges.push_back(edge.get<std::string>());
        }
    }

    #ifdef TASK
    std::string taskId = influxDB::getTaskIDByName(std::string(nodePubToUpdate.publishesTo) + "_in");
    std::string taskPayload = influxDB::createPayloadForTask("STREAM", incomingEdges, nodePubToUpdate.publishesTo);
    (taskId == "") ? curl::push(taskPayload, curl::INFLUXDB_SETTASK) : curl::push(taskPayload, curl::INFLUXDB_UPDATETASK, taskId);
    #endif
    return nodePubToUpdate;
}

void handlePublishersUpdate(sharedMem::TraceMessage msg, std::vector<RequestingClient> &clients, const IpcServer &server, int pipeToRelationMgmt_w) {
    std::string payloadNeo4j = publisher::getPayload(msg.publisher.topicName, msg.publisher.nodeHandle, msg.publisher.publisherHandle);
    NodePublishersToUpdate nodePubToUpdate = queryGraphDbForPublisher(payloadNeo4j);

    if (!strstr(msg.subscriber.topicName, "/_action/")) {
        for (RequestingClient &client : clients) {
            if (client.primaryKey == nodePubToUpdate.primaryKey)
            {
                server.sendNodePublishersToUpdate(nodePubToUpdate, client.pid, false);
            }
        }

        TopicPublishersUpdate payloadTopic { .isUpdate = true };
        util::parseString(payloadTopic.primaryKey, nodePubToUpdate.publishesTo);
        util::parseString(payloadTopic.publisher, nodePubToUpdate.primaryKey);
        util::parseString(payloadTopic.edge, nodePubToUpdate.edge);
        for (RequestingClient &client : clients) {
            if (client.primaryKey == payloadTopic.primaryKey)
            {
                server.sendTopicPublishersUpdate(payloadTopic, client.pid, false);
            }
        }
    }

    pipe_ns::UnionResponse unionResp;
    std::memcpy(unionResp.topicResp.name, msg.publisher.topicName, sizeof(msg.publisher.topicName));
    pipe_ns::writeT<pipe_ns::UnionResponse>(pipeToRelationMgmt_w, unionResp, pipe_ns::MsgType::TOPICRESPONSE);
}

NodeResponse queryGraphDbForNode(std::string payloadNeo4j) {
    NodeResponse nodeResponse;

    std::string responseNeo4J = curl::push(payloadNeo4j, curl::NEO4J);

    nlohmann::json data = nlohmann::json::parse(responseNeo4J);
    if (!data["results"].empty() &&
        !data["results"][0]["data"].empty() && 
        !data["results"][0]["data"][0]["row"].empty())
        {
        util::parseString(nodeResponse.primaryKey, data["results"][0]["data"][0]["row"][0]["primaryKey"].get<std::string>());
    } else {
        std::cout << "Failed parsing JSON (wanted ID):" << std::endl;
        std::cout << payloadNeo4j << std::endl;
        std::cout << responseNeo4J << std::endl;
    }
    if (!data["results"].empty() &&
        !data["results"][0]["data"].empty() && 
        !data["results"][0]["data"][0]["row"].empty())
        {
        nodeResponse.bootCount          = data["results"][0]["data"][0]["row"][0]["bootcounter"];
        nodeResponse.stateChangeTime    = data["results"][0]["data"][0]["row"][0]["stateChangeTime"];
    } else {
        std::cout << "Failed parsing JSON (wanted ID)" << std::endl;
        std::cout << payloadNeo4j << std::endl;
        std::cout << responseNeo4J << std::endl;
    }

    return nodeResponse;
}

void handleNodeUpdate(sharedMem::TraceMessage msg, std::vector<RequestingClient> &clients, const IpcServer &server, int pipeToRelationMgmt_w) {
    std::string fqName = getFullName(msg.node.name, msg.node.nspace);
    std::string payloadNeo4j = node::getPayload(fqName, msg.node.handle, sharedMem::State::ACTIVE, msg.node.pid, msg.node.stateChangeTime);
    NodeResponse nodeResponse = queryGraphDbForNode(payloadNeo4j);

    nodeResponse.state              = sharedMem::State::ACTIVE;
    nodeResponse.pid                = msg.node.pid;
    nodeResponse.stateChangeTime    = msg.node.stateChangeTime;
    util::parseString(nodeResponse.name, fqName);


    for (RequestingClient &client : clients)
    {
        if (client.primaryKey == nodeResponse.primaryKey)
        {
            server.sendNodeResponse(nodeResponse, client.pid, false);
        }
    }

    pipe_ns::UnionResponse unionResp;
    unionResp.nodeResp = nodeResponse;
    pipe_ns::writeT<pipe_ns::UnionResponse>(pipeToRelationMgmt_w, unionResp, pipe_ns::MsgType::NODERESPONSE);
}

NodeTimerToUpdate queryGraphDbForTimer(std::string payloadNeo4j) {
    NodeTimerToUpdate nodeTimerToUpdate;

    std::string responseNeo4J = curl::push(payloadNeo4j, curl::NEO4J);

    nlohmann::json data = nlohmann::json::parse(responseNeo4J);

    if (!data["results"].empty() &&
        !data["results"][0]["data"].empty() && 
        !data["results"][0]["data"][0]["row"].empty()) {
            util::parseString(nodeTimerToUpdate.primaryKey, data["results"][0]["data"][0]["row"][0].get<std::string>());
    } else {
        std::cout << "Failed parsing JSON (wanted ID):" << std::endl;
        std::cout << payloadNeo4j << std::endl;
        std::cout << responseNeo4J << std::endl;
    }

    return nodeTimerToUpdate;
}

void handleTimerToUpdate(sharedMem::TraceMessage msg, std::vector<RequestingClient> &clients, const IpcServer &server) {
    std::string payloadNeo4j = timer::getPayload(msg.timer.nodeHandle, msg.timer.frequency);
    NodeTimerToUpdate nodeTimerToUpdate = queryGraphDbForTimer(payloadNeo4j);
    nodeTimerToUpdate.frequency = msg.timer.frequency;

    for (RequestingClient &client : clients) {
        if (client.primaryKey == nodeTimerToUpdate.primaryKey) {
            server.sendNodeTimerToUpdate(nodeTimerToUpdate, client.pid, false);
        }
    }
}

void handleStateMachineInit(sharedMem::TraceMessage msg) {
    std::string payloadNeo4j = node::getPayloadSetStateMachine(msg.lcSmInit.nodeHandle, msg.lcSmInit.stateMachine, sharedMem::State::UNCONFIGURED);
   
    curl::push(payloadNeo4j, curl::NEO4J);
}

NodeStateUpdate queryGraphDbForStateChange(std::string payloadNeo4j) {
    NodeStateUpdate nodeStateUpdate;

    std::string responseNeo4J = curl::push(payloadNeo4j, curl::NEO4J);

    nlohmann::json data = nlohmann::json::parse(responseNeo4J);
    if (!data["results"].empty() &&
        !data["results"][0]["data"].empty() && 
        !data["results"][0]["data"][0]["row"].empty()) {
            util::parseString(nodeStateUpdate.primaryKey, data["results"][0]["data"][0]["row"][0].get<std::string>());
    } else {
        std::cout << "Failed parsing JSON (wanted ID):" << std::endl;
        std::cout << payloadNeo4j << std::endl;
        std::cout << responseNeo4J << std::endl;
    }

    return nodeStateUpdate;
}

void handleStateTransitionToUpdate(sharedMem::TraceMessage msg, std::vector<RequestingClient> &clients, const IpcServer &server) {
    std::string payloadNeo4j = node::getPayloadSetStateTransition(msg.lcTrans.stateMachine, msg.lcTrans.state, msg.lcTrans.stateChangeTime);
    NodeStateUpdate nodeStateUpdate = queryGraphDbForStateChange(payloadNeo4j);
    nodeStateUpdate.state = msg.lcTrans.state;
    nodeStateUpdate.stateChangeTime = msg.lcTrans.stateChangeTime;

    for (RequestingClient &client : clients) {
        if (client.primaryKey == nodeStateUpdate.primaryKey) {
            server.sendNodeStateUpdate(nodeStateUpdate, client.pid, false);
        }
    }
}

std::vector<NodeStateUpdate> extractNodeInfo(std::string response) {
    std::vector<NodeStateUpdate> nodeStateUpdates;

    nlohmann::json data = nlohmann::json::parse(response);
    if (!data["results"].empty() &&
        !data["results"][0]["data"].empty() && 
        !data["results"][0]["data"][0]["row"].empty()) {
            nlohmann::json row = data["results"][0]["data"][0]["row"];
            for (auto item : row) {
                NodeStateUpdate nodeStateUpdate;
                if (item.contains("primaryKey")) {
                    util::parseString(nodeStateUpdate.primaryKey, item["primaryKey"].get<std::string>());
                }
                if (item.contains("state")) {
                    nodeStateUpdate.state = item["state"];
                }
                if (item.contains("stateChangeTime")) {
                    nodeStateUpdate.stateChangeTime = item["stateChangeTime"];
                }
                nodeStateUpdates.push_back(nodeStateUpdate);
            }
    } else {
        std::cout << "Failed parsing JSON:" << std::endl;
        std::cout << response << std::endl;
    }

    return nodeStateUpdates;
}

void setNodeOffline( NodeResponse nodeUpdate, std::vector<RequestingClient> &clients, const IpcServer &server) {
    std::string payload = node::getPayloadSetNodeStateByPID(nodeUpdate.pid, nodeUpdate.stateChangeTime, sharedMem::State::INACTIVE);
    std::string response = curl::push(payload, curl::NEO4J);
    std::vector<NodeStateUpdate> NodeStateUpdates = extractNodeInfo(response);

    for (NodeStateUpdate nodeStateUpdate : NodeStateUpdates) {
        for (RequestingClient &client : clients) {
            if (client.primaryKey == nodeStateUpdate.primaryKey) {
                server.sendNodeStateUpdate(nodeStateUpdate, client.pid, false);
            }
        }

        #ifdef TASK
        std::string taskId = influxDB::getTaskIDByName(std::string(nodeStateUpdate.primaryKey) + "_in");
        if (taskId != "") curl::push(curl::INFLUXDB_DELETETASK, taskId);
        #endif
    }   
}

void handleClient(RequestingClient &client, std::vector<RequestingClient> &clients) {
    std::cout << "pid:   " << client.pid
    << "\n\trequestId:  " << client.requestId <<
        "\n\tprimaryKey: " << client.primaryKey <<
        "\n\tupdates:    " << client.updates << std::endl;

    if (client.updates) {
        bool contains = false;
        for (RequestingClient &item : clients)
        {
            if (item.pid == client.pid &&
                item.requestId == client.requestId)
            {
                contains = true;
                break;
            }
        }
        if (!contains)
            clients.push_back(client);
    } else {
        for (size_t i = 0; i < clients.size(); i++) {
            if (clients[i].pid == client.pid &&
                clients[i].requestId == client.requestId) {
                clients.erase(clients.begin() + i);
                break;
            }
        }
    }
    std::cout << "nrOf supped Clients: " << clients.size() << std::endl;
}

bool handleSearchRequests(const IpcServer &server) {
    requestId_t requestId;
    pid_t pid;
    std::optional<SearchRequest> search = server.receiveSearchRequest(requestId, pid, false);
    if (search.has_value()) {
        SearchRequest payload = search.value();
        std::string request = (payload.type == payload.NODE) ? node::getPayloadSearch(payload.name) : topic::getPayloadSearch(payload.name);
        std::string response = curl::push(request, curl::NEO4J);

        json data = nlohmann::json::parse(response);
        json row = data["results"][0]["data"][0]["row"];
        SearchResponse searchResp;
        if (!row.empty() && !row[0].empty()) {
            util::parseString(searchResp.primaryKey, row[0].get<std::string>());
        } else {
            util::parseString(searchResp.primaryKey, "");
        }
        server.sendSearchResponse(searchResp, pid, false);
        
        return true;        
    }
    
    return false;
}

void listenToStructuralTracer(sharedMem::SHMChannel<sharedMem::TraceMessage> &channel, std::vector<RequestingClient> &clients, const IpcServer &server, std::map<Module_t, pipe_ns::Pipe> pipes) {
    while (gsRunning) {
        sharedMem::TraceMessage msg(sharedMem::MessageType::NONE);
        if (channel.receive(msg, true)) {
            switch (msg.header.type) {
                case sharedMem::MessageType::NODETRACE:
                    handleNodeUpdate(msg, clients, server, pipes[RELATIONMGMT].write);
                    break;
                case sharedMem::MessageType::PUBLISHERTRACE:
                    handlePublishersUpdate(msg, clients, server, pipes[RELATIONMGMT].write);
                    break;
                case sharedMem::MessageType::SUBSCRIBERTRACE:
                    handleSubscribersUpdate(msg, clients, server, pipes[RELATIONMGMT].write);
                    break;
                case sharedMem::MessageType::SERVICETRACE:
                    if (strstr(msg.service.name, "/_action/")) {
                        handleActionServerForUpdate(msg, clients, server);
                        break;
                    }
                    handleIsServerForUpdate(msg, clients, server);
                    break;
                case sharedMem::MessageType::CLIENTTRACE:
                    if (strstr(msg.client.srvName, "/_action/")) {
                        handleActionClientToUpdate(msg, clients, server);
                        break;
                    }
                    handleIsClientToUpdate(msg, clients, server);
                    break;
                case sharedMem::MessageType::TIMERTRACE:
                    handleTimerToUpdate(msg, clients, server);
                    break;
                case sharedMem::MessageType::STATEMACHINEINITTRACE:
                    handleStateMachineInit(msg);
                    break;
                case sharedMem::MessageType::STATETRANSITIONTRACE:
                    handleStateTransitionToUpdate(msg, clients, server);
                    break;
                default: 
                    std::cout << "unknown Type" << std::endl;
                    break;
            }
        }
    }
    std::cout << "channel behind" << std::endl;

}


void nodeAndTopicObserver(const IpcServer &server, std::map<Module_t, pipe_ns::Pipe> pipes, std::atomic<bool> &running) {
    std::cout << "started nodeAndTopicObserver" << std::endl;
    int pipe_r = pipes[MAIN].read;

    std::vector<RequestingClient> clients;
    std::vector<pid_t> pids;

    sharedMem::SHMChannel<sharedMem::TraceMessage> channel("/babeltonato");
    std::thread structuralListener = std::thread(listenToStructuralTracer, std::ref(channel), std::ref(clients), std::cref(server), pipes);
    std::cout << "startet listening" << std::endl;
    RequestingClient client;
    auto then = std::chrono::steady_clock::now();
    while (gsRunning) {
        bool receivedMessage = false;

        bool gotRequest;
        do {
            gotRequest = false;
            {
                std::optional<NodeRequest> request = server.receiveNodeRequest(client.requestId, client.pid, false);
                if (request.has_value()) {
                    NodeRequest payload = request.value();
                    client.primaryKey   = payload.primaryKey;
                    client.updates      = payload.updates;

                    if (client.updates)  handleClient(client, clients);

                    singleTimeNodeResponse(server, client, payload.primaryKey);
                    gotRequest = true;
                }
            }
            {
                std::optional<TopicRequest> request = server.receiveTopicRequest(client.requestId, client.pid, false);
                if (request.has_value()) {
                    TopicRequest payload = request.value();
                    client.primaryKey   = payload.primaryKey;
                    client.updates      = payload.updates;
                    
                    if (client.updates) handleClient(client, clients);

                    singleTimeTopicResponse(server, client, payload.primaryKey);
                    gotRequest = true;
                }
            }
        } while (gotRequest);

        NodeResponse nodeUpdate;
        ssize_t ret = pipe_ns::readT<NodeResponse>(pipes[PROCESSOBSERVER].read, nodeUpdate);
        if (ret != -1) {
            setNodeOffline(nodeUpdate, clients, server);
            continue;
        }

        if (handleSearchRequests(server)) continue;

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now-then);
        auto sleepTime = 1s - elapsed;
        if (sleepTime > 0s) {
            std::this_thread::sleep_for(sleepTime);
        }
        then = std::chrono::steady_clock::now();
    }

    channel.~SHMChannel();
    std::cout << "finalized NodeTopicObserver" << std::endl;
}