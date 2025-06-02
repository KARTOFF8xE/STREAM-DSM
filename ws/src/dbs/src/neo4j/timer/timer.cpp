#include <neo4j/timer/timer.hpp>

#include <fmt/core.h>


namespace timer {

    std::string getPayload(u_int64_t nodeHandle, u_int32_t frequency);


    std::string getPayload(u_int64_t nodeHandle, u_int32_t frequency) {
        return fmt::format(R"(
            {{ 
                "statements":
                    [
                        {{ "statement": "MATCH (n:Active {{handle: $nodeHandle}}) MERGE (n) -[t:timer {{frequency: $frequency}}]-> (n) ON CREATE SET t.primaryKey=randomUUID() SET t.active=true WITH n RETURN n.primaryKey ",
                        "parameters": {{
                            "nodeHandle": {},
                            "frequency": {}
                            }}
                        }}
                    ]
            }}
        )", nodeHandle, frequency);
    }

}