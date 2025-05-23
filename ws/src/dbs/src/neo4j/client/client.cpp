#include <neo4j/client/client.hpp>

#include <string>
#include <fmt/core.h>


namespace client {

    std::string getPayload(std::string name, u_int64_t handle) {
        return fmt::format(R"(
            {{
                "statements":
                    [
                        {{ "statement": "MATCH (n:Node {{handle: $node_handle}}) SET n.Clients = COALESCE(n.Clients, []) + $name WITH n MATCH (s:Node) WHERE $name IN s.Services CREATE (s)-[:service_for {{name: $name}}]->(n) RETURN COLLECT(DISTINCT {{ node_id: toInteger(split(elementId(n), \":\")[-1]), server_id: toInteger(split(elementId(s), \":\")[-1]) }}) AS row ",
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