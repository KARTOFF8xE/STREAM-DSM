#include <neo4j/service/service.hpp>

#include <string>
#include <fmt/core.h>


namespace service {

    std::string getPayload(std::string serviceName, u_int64_t handle) {
        return fmt::format(R"(
            {{
                "statements":
                    [
                        {{ "statement": "MATCH (n:Active {{handle: $nodeHandle}}) SET n.Services = COALESCE(n.Services, []) + $name WITH n MATCH (c:Active) WHERE $name IN c.Clients CREATE (n)-[:sending {{serviceName: $name, mode: \"responds\"}}]->(c), (c)-[:sending {{serviceName: $name, mode: \"requests\"}}]->(n) RETURN COLLECT(DISTINCT {{ client_id: toInteger(split(elementId(c), \":\")[-1]), node_id: toInteger(split(elementId(n), \":\")[-1]) }}) AS row ",
                        "parameters": {{
                            "name": "{}",
                            "nodeHandle": {}
                            }}
                        }}
                    ]
            }}
            )", serviceName, handle);
    }

    std::string getPayload(std::string serviceName, u_int64_t handle, std::string actionName) {
        std::string adder;
        if (serviceName == "send_goal") adder = "+ $actionName";
        return fmt::format(R"(
            {{
                "statements":
                    [
                        {{ "statement": "MATCH (n:Active {{handle: $nodeHandle}}) SET n.Actions = COALESCE(n.Actions, []) {} WITH n MATCH (c:Active) WHERE $actionName IN c.ActionClients CREATE (n)-[:sending {{serviceName: $name, mode: \"responds\", actionName: $actionName}}]->(c), (c)-[:sending {{serviceName: $name, mode: \"requests\", actionName: $actionName}}]->(n) RETURN COLLECT(DISTINCT {{ client_id: toInteger(split(elementId(c), \":\")[-1]), node_id: toInteger(split(elementId(n), \":\")[-1]) }}) AS row ",
                        "parameters": {{
                            "name": "{}",
                            "nodeHandle": {},
                            "actionName": "{}"
                            }}
                        }}
                    ]
            }}
            )", adder, serviceName, handle, actionName);
    }

}