#include <neo4j/actionservice/actionservice.hpp>

#include <string>
#include <fmt/core.h>


namespace actionservice {

    std::string getPayload(std::string serviceName, u_int64_t handle, std::string actionName) {
        std::string adder;
        if (serviceName == "send_goal") adder = "+ $actionName";
        return fmt::format(R"(
            {{
                "statements":
                    [
                        {{ "statement": "MATCH (n:Active {{handle: $nodeHandle}}) SET n.Actions = COALESCE(n.Actions, []) {} WITH n MATCH (c:Active) WHERE $actionName IN c.ActionClients MERGE (n)-[s:sending {{serviceName: $name, mode: \"responds\", actionName: $actionName}}]->(c) SET s.active=true WITH n,c MERGE (c)-[s:sending {{serviceName: $name, mode: \"requests\", actionName: $actionName}}]->(n) SET s.active=true WITH n,c RETURN COLLECT(DISTINCT {{ client_id: toInteger(split(elementId(c), \":\")[-1]), node_id: toInteger(split(elementId(n), \":\")[-1]) }}) AS row ",
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