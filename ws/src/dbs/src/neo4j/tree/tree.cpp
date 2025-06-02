#include <neo4j/tree/tree.hpp>

#include <string>
#include <fmt/core.h>


namespace tree {

    std::string getPayloadForTree(std::string root, Tree tree) {
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
                    "statement": "MATCH (root {{primaryKey: $primKey}}) WITH root MATCH path = (root)-[:{}*0..]->(n) RETURN collect(n.primaryKey) AS primaryKeys",
                    "parameters": {{
                        "primKey": "{}"
                    }}
                }}
            ]
        }}
        )", edge, root);

    }

}
