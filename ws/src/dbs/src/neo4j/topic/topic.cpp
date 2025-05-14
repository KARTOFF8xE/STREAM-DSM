#include <neo4j/topic/topic.hpp>

#include <string>
#include <fmt/core.h>


namespace topic {

    std::string getPayloadRequestByPrimaryKey(pid_t pid) {
        return fmt::format(R"(
            {{
                "statements":
                    [
                        {{ "statement": "MATCH (n) WITH n, toInteger(last(SPLIT(elementId(n), \":\"))) AS extractedId WHERE extractedId = $primaryKey OPTIONAL MATCH (n)-[:subscription]->(subscriber) WITH n, extractedId, collect(toInteger(last(SPLIT(elementId(subscriber), \":\")))) AS subscribers OPTIONAL MATCH (publisher)-[:publishes_to]->(n) WITH n, extractedId, subscribers, collect(toInteger(last(SPLIT(elementId(publisher), \":\")))) AS publishers RETURN {{ id: extractedId, name: n.name, publishers: publishers, subscribers: subscribers }} AS result ",
                        "parameters": {{
                            "primaryKey": {}
                            }}
                        }}
                    ]
            }}
        )", pid);
    }

}