#include <neo4j/tree/tree.hpp>

#include <string>
#include <fmt/core.h>


namespace tree {

    std::string getPayloadForTree(primaryKey_t root, Tree tree) {
        std::string edge;
        switch (tree)
        {
        case NAMESPACEDRIVEN:   edge = "namespace"; break;
        case PROCESSDRIVEN:     edge = "process"; break;
        default:                edge = "foo"; break;
        }

        return fmt::format(R"(
        {{
            "statements": [
                {{
                    "statement": "MATCH (root) WITH root, toInteger(last(SPLIT(elementId(root), \":\"))) AS extractedId WHERE extractedId = $primKey MATCH path = (root)-[:{}*0..]->(n) RETURN collect(toInteger(last(SPLIT(elementId(n), \":\")))) AS primaryKeys",
                    "parameters": {{
                        "primKey": {}
                    }}
                }}
            ]
        }}
        )", edge, root);

    }

}
