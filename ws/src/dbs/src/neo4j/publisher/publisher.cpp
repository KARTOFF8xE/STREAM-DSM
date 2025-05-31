#include <neo4j/publisher/publisher.hpp>

#include <string>
#include <fmt/core.h>


namespace publisher {

    std::string getPayload(std::string name, u_int64_t nodeHandle, u_int64_t pubHandle) {
        return fmt::format(R"(
            {{
                "statements":
                    [
                        {{ "statement": "MATCH (n:Active {{handle: $nodeHandle}}) MERGE (t:Passive {{name: $name}}) MERGE (n)-[p:publishing]->(t) SET p.handle=$pubHandle, p.active=true RETURN {{node_id: toInteger(split(elementId(n), \":\")[-1]), topic_id: toInteger(split(elementId(t), \":\")[-1])}} AS row ",
                        "parameters": {{
                            "name": "{}",
                            "nodeHandle": {},
                            "pubHandle": {}
                            }}
                        }}
                    ]
            }}
        )", name, nodeHandle, pubHandle);
    }

    std::string getprimKeyByPubHandle(u_int64_t pubHandle) {
        return fmt::format(R"(
            {{
                "statements":
                    [
                        {{ "statement": "MATCH ()-[p:publishing {{handle: $pubHandle}}]->() RETURN toInteger(last(SPLIT(elementId(p), \":\"))) ",
                        "parameters": {{
                            "pubHandle": {}
                            }}
                        }}
                    ]
            }}
        )",pubHandle);
    }

}