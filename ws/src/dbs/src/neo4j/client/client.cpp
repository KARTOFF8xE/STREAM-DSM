#include <neo4j/client/client.hpp>

#include <string>
#include <fmt/core.h>


namespace client {

    std::string getPayload(std::string serviceName, u_int64_t nodeHandle) {
        return fmt::format(R"(
            {{
                "statements":
                    [
                        {{ "statement": "MATCH (n:Active {{handle: $nodeHandle}}) SET n.Clients = COALESCE(n.ServiceClients, []) + $name WITH n MATCH (s:Active) WHERE $name IN s.Services MERGE (s)-[a:sending {{serviceName: $name, mode: \"responds\"}}]->(n) ON CREATE SET a.primaryKey=randomUUID() SET a.active=true WITH n,s MERGE (n)-[a:sending {{serviceName: $name, mode: \"requests\"}}]->(s) ON CREATE SET a.primaryKey=randomUUID() SET a.active=true WITH n,s RETURN COLLECT(DISTINCT {{ node_id: n.primaryKey, server_id: s.primaryKey }}) AS row ",
                        "parameters": {{
                            "name": "{}",
                            "nodeHandle": {}
                            }}
                        }}
                    ]
            }}
        )", serviceName, nodeHandle);
    }

}