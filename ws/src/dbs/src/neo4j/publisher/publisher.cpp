#include <neo4j/publisher/publisher.hpp>

#include <string>
#include <fmt/core.h>


namespace publisher {

    std::string getPayload(std::string name, u_int64_t handle) {
        return fmt::format(R"(
            {{
                "statements":
                    [
                        {{ "statement": "MATCH (n:Active {{handle: $node_handle}}) MERGE (t:Passive {{name: $name}}) CREATE (n)-[:publishes_to]->(t) RETURN {{node_id: toInteger(split(elementId(n), \":\")[-1]), topic_id: toInteger(split(elementId(t), \":\")[-1])}} AS row ",
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