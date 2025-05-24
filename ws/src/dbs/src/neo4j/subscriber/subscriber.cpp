#include <neo4j/subscriber/subscriber.hpp>

#include <string>
#include <fmt/core.h>


namespace subscriber {

    std::string getPayload(std::string name, u_int64_t handle) {
        return fmt::format(R"(
            {{
                "statements":
                    [
                        {{ "statement": "MATCH (n:Active {{handle: $node_handle}}) MERGE (t:Passive {{name: $name}}) CREATE (n)<-[:subscription]-(t) RETURN {{node_id: toInteger(split(elementId(n), \":\")[-1]), topic_id: toInteger(split(elementId(t), \":\")[-1])}} AS row ",
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