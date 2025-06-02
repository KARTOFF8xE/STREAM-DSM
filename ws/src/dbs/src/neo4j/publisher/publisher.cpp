#include <neo4j/publisher/publisher.hpp>

#include <string>
#include <fmt/core.h>


namespace publisher {

    std::string getPayload(std::string name, u_int64_t nodeHandle, u_int64_t pubHandle) {
        return fmt::format(R"(
            {{
                "statements":
                    [
                        {{ "statement": "MATCH (n:Active {{handle: $nodeHandle}}) MERGE (t:Passive {{name: $name}}) ON CREATE SET t.primaryKey=randomUUID() MERGE (n)-[p:publishing]->(t) ON CREATE SET p.primaryKey=randomUUID() SET p.handle=$pubHandle, p.active=true WITH p,t,n MATCH ()-[p2:publishing]->(t) WITH p,t,n,collect(p2.primaryKey) AS incomingEdges RETURN {{node_id: n.primaryKey, topic_id: t.primaryKey, incomingEdges: incomingEdges}} AS row ",
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
                        {{ "statement": "MATCH ()-[p:publishing {{handle: $pubHandle}}]->() RETURN p.primaryKey ",
                        "parameters": {{
                            "pubHandle": {}
                            }}
                        }}
                    ]
            }}
        )",pubHandle);
    }

}