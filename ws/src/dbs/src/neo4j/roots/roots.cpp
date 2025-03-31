#include <neo4j/roots/roots.hpp>

#include <string>
#include <fmt/core.h>


namespace createRoot {

    std::string getPayloadCreateNameSpaceAndLinkPassiveHelpers(std::string fullName) {
        return fmt::format(R"(
            {{
                "statements":
                    [
                        {{ "statement": "WITH $name AS path WITH split(path, \"/\")[1..] AS parts WITH range(0, size(parts)-1) AS indices, parts UNWIND indices AS i WITH \"/\" + reduce(s = \"\", p IN parts[0..i+1] | CASE WHEN s = \"\" THEN p ELSE s + \"/\" + p END) AS folderPath, CASE WHEN i = 0 THEN \"/\" ELSE \"/\" + reduce(s = \"\", p IN parts[0..i] | CASE WHEN s = \"\" THEN p ELSE s + \"/\" + p END) END AS parentPath MERGE (f:Node {{name: folderPath}}) MERGE (parent:Node {{name: parentPath}}) MERGE (parent)-[:namespace]->(f) ",
                        "parameters": {{
                            "name": "{}"
                            }}
                        }}
                    ]
            }}
        )", fullName);
    }

    std::string getPayloadCreateProcessAndLinkPassiveHelpers(std::string fullName) {
        return fmt::format(R"(
            {{
                "statements":
                    [
                        {{ "statement": "UNWIND $pids AS pidName MERGE (n:Node {{pid: pidName.pid}}) ON CREATE SET n.name = pidName.name WITH collect(n) AS nodes UNWIND range(0, size(nodes)-2) AS i WITH nodes[i] AS child, nodes[i+1] AS parent, nodes MERGE (parent)-[:process]->(child) WITH nodes UNWIND nodes AS n RETURN DISTINCT n.pid AS pid, last(split(elementId(n), ':')) AS primaryKey ",
                        "parameters": {{
                            "pids": {}
                            }}
                        }}
                    ]
            }}
        )", fullName);
    }
    
}