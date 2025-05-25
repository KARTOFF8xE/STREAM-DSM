#include <neo4j/client/client.hpp>

#include <string>
#include <fmt/core.h>


namespace client {

    std::string getPayload(std::string serviceName, u_int64_t nodeHandle) {
        return fmt::format(R"(
            {{
                "statements":
                    [
                        {{ "statement": "MATCH (n:Active {{handle: $nodeHandle}}) SET n.Clients = COALESCE(n.ServiceClients, []) + $name WITH n MATCH (s:Active) WHERE $name IN s.Services CREATE (s)-[:sending {{serviceName: $name, mode: \"responds\"}}]->(n), (n)-[:sending {{serviceName: $name, mode: \"requests\"}}]->(s) RETURN COLLECT(DISTINCT {{ node_id: toInteger(split(elementId(n), \":\")[-1]), server_id: toInteger(split(elementId(s), \":\")[-1]) }}) AS row ",
                        "parameters": {{
                            "name": "{}",
                            "nodeHandle": {}
                            }}
                        }}
                    ]
            }}
        )", serviceName, nodeHandle);
    }

    std::string getPayload(std::string serviceName, u_int64_t nodeHandle, std::string actionName) {
        std::string adder;
        if (serviceName == "send_goal") adder = "+ $actionName";
        return fmt::format(R"(
            {{
                "statements":
                    [
                        {{ "statement": "MATCH (n:Active {{handle: $nodeHandle}}) SET n.ActionClients = COALESCE(n.ActionClients, []) {} WITH n MATCH (s:Active) WHERE $actionName IN s.Actions CREATE (s)-[:sending {{serviceName: $name, mode: \"responds\", actionName: $actionName}}]->(n), (n)-[:sending {{serviceName: $name, mode: \"requests\", actionName: $actionName}}]->(s) RETURN COLLECT(DISTINCT {{ node_id: toInteger(split(elementId(n), \":\")[-1]), server_id: toInteger(split(elementId(s), \":\")[-1]) }}) AS row ",
                        "parameters": {{
                            "name": "{}",
                            "nodeHandle": {},
                            "actionName": "{}"
                            }}
                        }}
                    ]
            }}
        )", adder, serviceName, nodeHandle, actionName);
    }

}