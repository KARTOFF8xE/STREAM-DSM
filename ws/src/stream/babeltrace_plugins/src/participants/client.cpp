#include <fmt/core.h>
#include <string>
#include <iostream>

#include <nlohmann/json.hpp>

#include <neo4j/client/client.hpp>
#include <neo4j/actionclient/actionclient.hpp>
#include <curl/myCurl.hpp>
#include <ipc/util.hpp>

#include "interface.hpp"
#include "participants/client.hpp"

using json = nlohmann::json;


void Client::extractInfo(const bt_event *event) {
    const bt_field *payload_field = bt_event_borrow_payload_field_const(event);
    const bt_field_class *field_class = bt_field_borrow_class_const(payload_field);
    if (bt_field_class_get_type(field_class) == BT_FIELD_CLASS_TYPE_STRUCTURE) {
        const bt_field *field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "service_name");
        this->name = std::string(bt_field_string_get_value(field));
        field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "node_handle");
        this->node_handle = bt_field_integer_unsigned_get_value(field);

        if (this->name.find("/_action/") != std::string::npos) this->isAction = true;
        const std::string ext("/_action/send_goal");
        if (this->name != ext &&
            this->name.size() > ext.size() &&
            this->name.substr(this->name.size() - ext.size()) == "/_action/send_goal") {
                this->name = this->name.substr(0, this->name.size() - ext.size());
        }
    } else { printf("\033[33;1WRONG TYPE\033[0m\n"); }
}

std::string Client::getPayload() {
    if (this->isAction) return actionclient::getPayload(this->name, this->node_handle);

    if (this->name.find("/_action/") != std::string::npos) return "";

    return client::getPayload(this->name, this->node_handle);
}

void Client::toGraph(std::string payload) {
    if (payload == "") return;

    std::string response = curl::push(payload);
    
    json data = nlohmann::json::parse(response);
    json row = data["results"][0]["data"][0]["row"];
    if (!row.empty() && !row[0].empty() && !row[0][0]["node_id"].empty())   this->primaryKey = row[0][0]["node_id"];
    
    size_t counter = 0;
    while (!row.empty() &&
        !row[0].empty() &&
        !row[0][counter].empty() &&
        !row[0][counter]["server_id"].empty()
        ) {
        this->server_primaryKeys.push_back(row[0][counter]["server_id"]);
        counter++;
    }
}

void Client::response(Communication &communication) {
    if (!this->isAction) {
        for (primaryKey_t item : this->server_primaryKeys) {
            NodeIsClientOfUpdate msg {
                .primaryKey     = this->primaryKey,
                .serverNodeId   = item,
                .isUpdate       = true,
            };
            util::parseString(msg.srvName, this->name);
            
            communication.server.sendNodeIsClientOfUpdate(msg, communication.pid, false);
        }
    } else {
        if (this->name.find("/_action/") != std::string::npos) return;
        for (primaryKey_t item : this->server_primaryKeys) {
            NodeIsActionClientOfUpdate msg {
                .primaryKey         = this->primaryKey,
                .actionserverNodeId = item,
                .isUpdate           = true,
            };
            util::parseString(msg.srvName, this->name);

            communication.server.sendNodeIsActionClientOfUpdate(msg, communication.pid, false);
        }
    }
}