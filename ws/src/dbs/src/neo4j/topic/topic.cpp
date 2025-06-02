#include <neo4j/topic/topic.hpp>

#include <string>
#include <fmt/core.h>


namespace topic {

    std::string getPayloadRequestByPrimaryKey(std::string primaryKey) {
        return fmt::format(R"(
            {{
                "statements":
                    [
                        {{ "statement": "MATCH (n:Passive {{primaryKey: $primaryKey}}) OPTIONAL MATCH (n)-[:subscribing]->(subscriber) WITH n, collect(subscriber.primaryKey) AS subscribers OPTIONAL MATCH (publisher)-[:publishing]->(n) WITH n, subscribers, collect(publisher.primaryKey) AS publishers RETURN {{ id: n.primaryKey, name: n.name, publishers: publishers, subscribers: subscribers }} AS result ",
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