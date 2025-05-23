#include <neo4j/service/service.hpp>

#include <string>
#include <fmt/core.h>


namespace service {

    std::string getPayload(std::string name, u_int64_t handle) {
        return fmt::format(R"(
            {{
                "statements":
                    [
                        {{ "statement": "MATCH (n:Node {{handle: $node_handle}}) SET n.Services = COALESCE(n.Services, []) + $name WITH n MATCH (c:Node) WHERE $name IN c.Clients CREATE (n)-[:service_for {{name: $name}}]->(c) RETURN COLLECT(DISTINCT {{ client_id: toInteger(split(elementId(c), \":\")[-1]), node_id: toInteger(split(elementId(n), \":\")[-1]) }}) AS row ",
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