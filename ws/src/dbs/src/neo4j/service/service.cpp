#include <neo4j/service/service.hpp>

#include <string>
#include <fmt/core.h>


namespace service {

    std::string getPayload(std::string serviceName, u_int64_t handle) {
        return fmt::format(R"(
            {{
                "statements":
                    [
                        {{ "statement": "MATCH (n:Active {{handle: $nodeHandle}}) SET n.Services = COALESCE(n.Services, []) + $name WITH n MATCH (c:Active) WHERE $name IN c.Clients WITH n, c MERGE (n)-[s:sending {{serviceName: $name, mode: \"responds\"}}]->(c) ON CREATE SET s.primaryKey=randomUUID() SET s.active=true WITH n,c MERGE (c)-[s:sending {{serviceName: $name, mode: \"requests\"}}]->(n) ON CREATE SET s.primaryKey=randomUUID() SET s.active=true WITH n,c RETURN COLLECT(DISTINCT {{ client_id: c.primaryKey, node_id: n.primaryKey }}) AS row ",
                        "parameters": {{
                            "name": "{}",
                            "nodeHandle": {}
                            }}
                        }}
                    ]
            }}
            )", serviceName, handle);
    }

}