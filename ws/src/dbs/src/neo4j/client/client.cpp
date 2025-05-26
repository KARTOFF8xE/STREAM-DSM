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

}