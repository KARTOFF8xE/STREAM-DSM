#include <neo4j/actionclient/actionclient.hpp>

#include <string>
#include <fmt/core.h>


namespace actionclient {

    std::string getPayload(std::string serviceName, u_int64_t nodeHandle, std::string actionName) {
        std::string adder;
        if (serviceName == "send_goal") adder = "+ $actionName";
        return fmt::format(R"(
            {{
                "statements":
                    [
                        {{ "statement": "MATCH (n:Active {{handle: $nodeHandle}}) SET n.ActionClients = COALESCE(n.ActionClients, []) {} WITH n MATCH (s:Active) WHERE $actionName IN s.Actions MERGE (s)-[a:sending {{serviceName: $name, mode: \"responds\", actionName: $actionName}}]->(n) ON CREATE SET a.primaryKey=randomUUID() SET a.active=true WITH n,s MERGE (n)-[a:sending {{serviceName: $name, mode: \"requests\", actionName: $actionName}}]->(s) ON CREATE SET a.primaryKey=randomUUID() SET a.active=true RETURN COLLECT(DISTINCT {{ node_id: n.primaryKey, server_id: s.primaryKey }}) AS row ",
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