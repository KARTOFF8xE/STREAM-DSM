#include <fmt/core.h>
#include <string>
#include <iostream>

#include "interface.h"
#include "participants/service.h"
#include "curl.h"

std::string Service::getPayload() {
    return fmt::format(R"(
    {{
        "statements":
            [
                {{ "statement": "MATCH (n:Node {{handle: $node_handle}}) SET n.Services = COALESCE(n.Services, []) + $name WITH n MATCH (c:Node) WHERE $name IN c.Clients CREATE (c)-[:requests]->(n) CREATE (n)-[:response]->(c) ",
                "parameters": {{
                    "name": "{}",
                    "node_handle": "{}"
                    }}
                }}
            ]
    }}
)", this->name, this->node_handle);
}

void Service::extractInfo(const bt_event *event) {
    const bt_field *payload_field = bt_event_borrow_payload_field_const(event);
    const bt_field_class *field_class = bt_field_borrow_class_const(payload_field);
    if (bt_field_class_get_type(field_class) == BT_FIELD_CLASS_TYPE_STRUCTURE) {
            const bt_field *field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "service_name");
            this->name = std::string(bt_field_string_get_value(field));
            field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "node_handle");
            this->node_handle = bt_field_integer_unsigned_get_value(field);

    } else { printf("\033[33;1WRONG TYPE\033[0m\n"); }
}

void Service::toGraph() {
    struct Request *request = new Request;

    request->username = "neo4j";
    request->password = "123456789";
    request->url = "http://172.17.0.1:7474/db/neo4j/tx/commit";

    request->query_request = getPayload();
    CURL *curl = getCurl(request);

    CURLcode res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        std::cerr << "Fehler bei der Anfrage: " << curl_easy_strerror(res) << std::endl;
    } else {
        // std::cout << "Antwort von Neo4j:" << std::endl;
        // std::cout << request->query_response << std::endl;
    }
}
