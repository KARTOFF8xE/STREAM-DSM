#include <neo4j/actionservice/actionservice.hpp>

#include <string>
#include <fmt/core.h>


namespace actionservice {

    std::string getPayload(std::string name, u_int64_t handle) {
        return fmt::format(R"(
            {{
                "statements":
                    [
                        {{ "statement": "MATCH (n:Node {{handle: $node_handle}}) SET n.ActionServices = COALESCE(n.ActionServices, []) + $name WITH n MATCH (c:Node) WHERE $name IN c.ActionClients CREATE (n)-[:action_for {{name: $name}}]->(c) RETURN COLLECT(DISTINCT {{ client_id: toInteger(split(elementId(c), \":\")[-1]), node_id: toInteger(split(elementId(n), \":\")[-1]) }}) AS row ",
                        "parameters": {{
                            "name": "{}",
                            "node_handle": {}
                            }}
                        }}
                    ]
            }}
            )", name, handle);
    }

}