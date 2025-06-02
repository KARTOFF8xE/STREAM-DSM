#include <neo4j/subscriber/subscriber.hpp>

#include <string>
#include <fmt/core.h>


namespace subscriber {

    std::string getPayload(std::string name, u_int64_t handle) {
        return fmt::format(R"(
            {{
                "statements":
                    [
                        {{ "statement": "MATCH (n:Active {{handle: $node_handle}}) MERGE (t:Passive {{name: $name}}) ON CREATE SET t.primaryKey=randomUUID() MERGE (n)<-[s:subscribing]-(t) ON CREATE SET s.primaryKey=randomUUID() SET s.active=true RETURN {{node_id: n.primaryKey, topic_id: t.primaryKey}} AS row ",
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