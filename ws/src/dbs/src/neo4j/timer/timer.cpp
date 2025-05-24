#include <neo4j/timer/timer.hpp>

#include <fmt/core.h>


namespace timer {

    std::string getPayload(u_int64_t nodeHandle, u_int32_t frequency);


    std::string getPayload(u_int64_t nodeHandle, u_int32_t frequency) {
        return fmt::format(R"(
            {{ 
                "statements":
                    [
                        {{ "statement": "MATCH (n:Active {{handle: $nodeHandle}}) CREATE (n) -[:timer {{frequency: $frequency}}]-> (n) WITH n RETURN toInteger(last(SPLIT(elementId(n), \":\"))) ",
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