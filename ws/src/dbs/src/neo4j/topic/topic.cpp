#include <neo4j/topic/topic.hpp>

#include <string>
#include <fmt/core.h>


namespace topic {

    std::string getPayloadRequestByPrimaryKey(std::string primaryKey) {
        return fmt::format(R"(
            {{
                "statements":
                    [
                        {{ "statement": "MATCH (n:Passive {{primaryKey: $primaryKey}}) OPTIONAL MATCH (n)-[sub:subscribing]->(subscriber) WHERE sub.primaryKey IS NOT NULL WITH n, collect({{node_id: subscriber.primaryKey, edge_id: sub.primaryKey}}) AS subscribers OPTIONAL MATCH (publisher)-[pub:publishing]->(n) WHERE pub.primaryKey IS NOT NULL WITH n, subscribers, collect({{node_id: publisher.primaryKey, edge_id: pub.primaryKey}}) AS publishers RETURN {{ id: n.primaryKey, name: n.name, publishers: publishers, subscribers: subscribers }} AS result ",
                        "parameters": {{
                            "primaryKey": "{}"
                            }}
                        }}
                    ]
            }}
        )", primaryKey);
    }

    std::string getPayloadSearch(std::string name) {
        return fmt::format(R"(
            {{ 
                "statements":
                    [
                        {{ "statement": "MATCH (n:Passive {{name: $name}}) RETURN n.primaryKey ",
                        "parameters": {{
                            "name": "{}"
                            }}
                        }}
                    ]
            }}
        )", name);
    }

}