#include <iostream>
#include <vector>
#include <nlohmann/json.hpp>

#include <ipc/ipc-client.hpp>
#include <ipc/util.hpp>
#include <curl/myCurl.hpp>
#include <neo4j/node/node.hpp>
#include <neo4j/topic/topic.hpp>

#include "datamgmt/nodeandtopicobserver/nodeandtopicobserver.hpp"
#include "datamgmt/relationmgmt/relationmgmt.hpp"
#include "datamgmt/datamgmt.hpp"
#include "pipe/pipe.hpp"


using json = nlohmann::json;


void singleTimeNodeResponse(IpcServer &server, Client client, primaryKey_t primaryKey) {
    std::string response = curl::push(node::getPayloadRequestByPrimaryKey(primaryKey), curl::neo4j);

    json payload = json::parse(response);
    json row = payload["results"][0]["data"][0]["row"];

    json requestedNode;
    std::vector<NodePublishersToUpdate>         pubs;
    std::vector<NodeSubscribersToUpdate>        subs;
    std::vector<NodeIsServerForUpdate>          isServerFor;
    std::vector<NodeIsClientOfUpdate>           isClientOf;
    std::vector<NodeIsActionServerForUpdate>    isActionServerFor;
    std::vector<NodeIsActionClientOfUpdate>     isActionClientOf;
    for (const json & item: row) {
        if (item.contains("name")) {
            requestedNode = item;
            continue;
        }

        size_t counter = 0;
        while (item[counter].contains("relationship")) {
            if (item[counter]["relationship"] == "publishes_to") {
                for (const json & node: item[counter]["nodes"]) {
                    pubs.push_back(
                        NodePublishersToUpdate {
                            .primaryKey     = primaryKey,
                            .publishesTo    = node["id"],
                            .isUpdate       = false,
                        }
                    );
                }
            }
            if (item[counter]["relationship"] == "subscription") {
                for (const json & node: item[counter]["nodes"]) {
                    subs.push_back(
                        NodeSubscribersToUpdate {
                            .primaryKey     = primaryKey,
                            .subscribesTo   = node["id"],
                            .isUpdate       = false,
                        }
                    );
                }
            }
            if (item[counter]["relationship"] == "service_for") {
                for (const json & node: item[counter]["nodes"]) {
                    if (node["direction"] == "outgoing") {
                        NodeIsServerForUpdate tmp {
                        .primaryKey     = primaryKey,
                        .clientNodeId   = node["id"],
                        .isUpdate       = false,
                        };
                        util::parseString(tmp.srvName, node["name"].get<std::string>());

                        isServerFor.push_back(tmp);
                    } else {
                        NodeIsClientOfUpdate tmp {
                            .primaryKey     = primaryKey,
                            .serverNodeId   = node["id"],
                            .isUpdate       = false,
                        };
                        util::parseString(tmp.srvName, node["name"].get<std::string>());

                        isClientOf.push_back(tmp);
                    }
                }
            }
            if (item[counter]["relationship"] == "action_for") {
                for (const json & node: item[counter]["nodes"]) {
                    if (node["direction"] == "outgoing") {
                        NodeIsActionServerForUpdate tmp {
                        .primaryKey         = primaryKey,
                        .actionclientNodeId = node["id"],
                        .isUpdate           = false,
                        };
                        util::parseString(tmp.srvName, node["name"].get<std::string>());

                        isActionServerFor.push_back(tmp);
                    } else {
                        NodeIsActionClientOfUpdate tmp {
                            .primaryKey         = primaryKey,
                            .actionserverNodeId = node["id"],
                            .isUpdate           = false,
                        };
                        util::parseString(tmp.srvName, node["name"].get<std::string>());

                        isActionClientOf.push_back(tmp);
                    }
                }
            }
            counter++;
        }
    }

    NodeResponse nodeResponse {
        .primaryKey         = primaryKey,
        .alive              = requestedNode["state"] > 0,
        .aliveChangeTime    = requestedNode["stateChangeTime"],
        .bootCount          = requestedNode["bootcounter"],
        .pid                = requestedNode["pid"],
        .nrOfInitialUpdates = pubs.size() + subs.size() +
            isClientOf.size() + isServerFor.size() +
            isActionClientOf.size() + isActionServerFor.size(),
    };
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
}

void singleTimeTopicResponse(IpcServer &server, Client client, primaryKey_t primaryKey) {
    std::string response = curl::push(topic::getPayloadRequestByPrimaryKey(primaryKey), curl::neo4j);

    json payload = json::parse(response);
    json row = payload["results"][0]["data"][0]["row"];

    std::vector<TopicPublishersUpdate>  pubs;
    std::vector<TopicSubscribersUpdate> subs;

    for (const primaryKey_t item : row[0]["publishers"]) {
        pubs.push_back(
            TopicPublishersUpdate {
                .primaryKey = primaryKey,
                .publisher = item,
                .isUpdate = false
            }
        );
    }
    for (const primaryKey_t item : row[0]["subscribers"]) {
        subs.push_back(
            TopicSubscribersUpdate {
                .primaryKey = primaryKey,
                .subscriber = item,
                .isUpdate = false
            }
        );
    }

    TopicResponse topicResponse {
        .primaryKey         = primaryKey,
        .nrOfInitialUpdates = pubs.size() + subs.size(),
    };
    util::parseString(topicResponse.name, row[0]["name"].get<std::string>());

    server.sendTopicResponse(topicResponse, client.pid);

    for (TopicPublishersUpdate item : pubs) {
        server.sendTopicPublishersUpdate(item, client.pid);
    }
    for (TopicSubscribersUpdate item : subs) {
        server.sendTopicSubscribersUpdate(item, client.pid);
    }
}

void nodeAndTopicObserver(const IpcServer &server, std::map<Module_t, Pipe> pipes, std::atomic<bool> &running) {
    std::cout << "started procObserver" << std::endl;
    int pipe_r = pipes[MAIN].read;
    // int pipe_writeToRelationMgmt = pipes[RELATIONMGMT].write;

    std::vector<Client> clients;
    std::vector<pid_t> pids;
    
    IpcClient ipcClient(2);
    requestId_t requestId;
    {
        NodeSwitchRequest msg = NodeSwitchRequest{
            .updates = true
        };
        ipcClient.sendNodeSwitchRequest(msg, requestId, false);
    }

    Client client;
    while (true) {
        ssize_t ret = -1;

        ret = readT<Client>(pipe_r, client);
        while (ret != -1) {
            handleClient(client, clients);
            ret = readT<Client>(pipe_r, client);
        };

        /*
        for (const pid_t &pid : pids) {
            if (kill(pid, 0) != 0) {
                std::cout << pid << " died." << std::endl;
                pids.erase(find(pids.begin(), pids.end(), pid));
                curl::push(node::getPayloadSetOffline(pid));
            }
        }
        */

        receiveNodeResponse(ipcClient, clients, server);
        receiveSubscribersToUpdate(ipcClient, clients, server);
        receivePublishersToUpdate(ipcClient, clients, server);
        receiveNodeIsServerForUpdate(ipcClient, clients, server);
        receiveNodeIsClientOfUpdate(ipcClient, clients, server);
        receiveNodeIsActionServerForUpdate(ipcClient, clients, server);
        receiveNodeIsActionClientOfUpdate(ipcClient, clients, server);
    }
    std::cout << "shutting down NodeTopicObserver" << std::endl;

    NodeSwitchRequest msg = NodeSwitchRequest{
        .updates = false
    };
    ipcClient.sendNodeSwitchRequest(msg, requestId, false);

    running.store(false);
}

void receiveNodeIsClientOfUpdate(IpcClient &ipcClient, std::vector<Client> &clients, const IpcServer &server) {
    std::optional<NodeIsClientOfUpdate> response = ipcClient.receiveNodeIsClientOfUpdate(false);
    if (response.has_value())
    {
        NodeIsClientOfUpdate payload = response.value();
        for (Client client : clients)
        {
            if (client.primaryKey == payload.primaryKey)
            {
                server.sendNodeIsClientOfUpdate(payload, client.pid, false);
            }
            if (client.primaryKey == payload.serverNodeId)
            {
                NodeIsServerForUpdate msg{
                    .primaryKey     = payload.serverNodeId,
                    .clientNodeId   = payload.primaryKey,
                    .isUpdate       = true,
                };
                util::parseString(msg.srvName, payload.srvName);

                server.sendNodeIsServerForUpdate(msg, client.pid, false);
            }
        }
    }
}

void receiveNodeIsServerForUpdate(IpcClient &ipcClient, std::vector<Client> &clients, const IpcServer &server) {
    std::optional<NodeIsServerForUpdate> response = ipcClient.receiveNodeIsServerForUpdate(false);
    if (response.has_value())
    {
        NodeIsServerForUpdate payload = response.value();
        for (Client client : clients)
        {
            if (client.primaryKey == payload.primaryKey)
            {
                server.sendNodeIsServerForUpdate(payload, client.pid, false);
            }
            if (client.primaryKey == payload.clientNodeId)
            {
                NodeIsClientOfUpdate msg{
                    .primaryKey     = payload.clientNodeId,
                    .serverNodeId   = payload.primaryKey,
                    .isUpdate       = true,
                };
                util::parseString(msg.srvName, payload.srvName);

                server.sendNodeIsClientOfUpdate(msg, client.pid, false);
            }
        }
    }
}

void receiveNodeIsActionClientOfUpdate(IpcClient &ipcClient, std::vector<Client> &clients, const IpcServer &server) {
    std::optional<NodeIsActionClientOfUpdate> response = ipcClient.receiveNodeIsActionClientOfUpdate(false);
    if (response.has_value())
    {
        NodeIsActionClientOfUpdate payload = response.value();
        for (Client client : clients)
        {
            if (client.primaryKey == payload.primaryKey)
            {
                server.sendNodeIsActionClientOfUpdate(payload, client.pid, false);
            }
            if (client.primaryKey == payload.actionserverNodeId)
            {
                NodeIsActionServerForUpdate msg{
                    .primaryKey         = payload.actionserverNodeId,
                    .actionclientNodeId = payload.primaryKey,
                    .isUpdate           = true,
                };
                util::parseString(msg.srvName, payload.srvName);

                server.sendNodeIsActionServerForUpdate(msg, client.pid, false);
            }
        }
    }
}

void receiveNodeIsActionServerForUpdate(IpcClient &ipcClient, std::vector<Client> &clients, const IpcServer &server) {
    std::optional<NodeIsActionServerForUpdate> response = ipcClient.receiveNodeIsActionServerForUpdate(false);
    if (response.has_value())
    {
        NodeIsActionServerForUpdate payload = response.value();
        for (Client client : clients)
        {
            if (client.primaryKey == payload.primaryKey)
            {
                server.sendNodeIsActionServerForUpdate(payload, client.pid, false);
            }
            if (client.primaryKey == payload.actionclientNodeId)
            {
                NodeIsActionClientOfUpdate msg{
                    .primaryKey         = payload.actionclientNodeId,
                    .actionserverNodeId = payload.primaryKey,
                    .isUpdate           = true,
                };
                util::parseString(msg.srvName, payload.srvName);

                server.sendNodeIsActionClientOfUpdate(msg, client.pid, false);
            }
        }
    }
}

void receiveSubscribersToUpdate(IpcClient &ipcClient, std::vector<Client> &clients, const IpcServer &server) {
    std::optional<NodeSubscribersToUpdate> response = ipcClient.receiveNodeSubscribersToUpdate(false);
    if (response.has_value())
    {
        NodeSubscribersToUpdate payloadNode = response.value();
        for (Client client : clients)
        {
            if (client.primaryKey == payloadNode.primaryKey)
            {
                server.sendNodeSubscribersToUpdate(payloadNode, client.pid, false);
            }
        }

        TopicSubscribersUpdate payloadTopic {
            .primaryKey = payloadNode.subscribesTo,
            .subscriber = payloadNode.primaryKey,
            .isUpdate = true
        };
        for (Client client : clients)
        {
            if (client.primaryKey == payloadTopic.primaryKey)
            {
                server.sendTopicSubscribersUpdate(payloadTopic, client.pid, false);
            }
        }
    }
}

void receivePublishersToUpdate(IpcClient &ipcClient, std::vector<Client> &clients, const IpcServer &server) {
    std::optional<NodePublishersToUpdate> response = ipcClient.receiveNodePublishersToUpdate(false);
    if (response.has_value())
    {
        NodePublishersToUpdate payloadNode = response.value();
        for (Client client : clients)
        {
            if (client.primaryKey == payloadNode.primaryKey)
            {
                server.sendNodePublishersToUpdate(payloadNode, client.pid, false);
            }
        }

        TopicPublishersUpdate payloadTopic {
            .primaryKey = payloadNode.publishesTo,
            .publisher = payloadNode.primaryKey,
            .isUpdate = true
        };
        for (Client client : clients)
        {
            if (client.primaryKey == payloadTopic.primaryKey)
            {
                server.sendTopicPublishersUpdate(payloadTopic, client.pid, false);
            }
        }
    }
}

void receiveNodeResponse(IpcClient &ipcClient, std::vector<Client> &clients, const IpcServer &server) {
    std::optional<NodeResponse> response = ipcClient.receiveNodeResponse(false);
    if (response.has_value())
    {
        NodeResponse payload = response.value();
        for (Client client : clients)
        {
            if (client.primaryKey == payload.primaryKey)
            {
                server.sendNodeResponse(payload, client.pid, false);
            }
        }

        relationMgmt::relationMgmt(payload);
    }
}

void handleClient(Client &client, std::vector<Client> &clients) {
    std::cout << "pid:   " << client.pid
    << "\n\trequestId:  " << client.requestId <<
        "\n\tprimaryKey: " << client.primaryKey <<
        "\n\tupdates:    " << client.updates << std::endl;

    if (client.updates)
    {
        bool contains = false;
        for (Client item : clients)
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
    }
    else
    {
        for (size_t i = 0; i < clients.size(); i++)
        {
            if (clients[i].pid == client.pid &&
                clients[i].requestId == client.requestId)
            {
                clients.erase(clients.begin() + i * sizeof(Client));
                break;
            }
        }
    }
    std::cout << "nrOf supped Clients: " << clients.size() << std::endl;
}