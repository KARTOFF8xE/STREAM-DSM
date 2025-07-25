#include <neo4j/roots/roots.hpp>

#include <string>
#include <fmt/core.h>


namespace createRoot {

    std::string getPayloadCreateNameSpaceAndLinkPassiveHelpers(std::string fullName) {
        return fmt::format(R"(
            {{
                "statements":
                    [
                        {{ "statement": "WITH $name AS path WITH split(path, \"/\")[1..] AS parts WITH range(0, size(parts)-1) AS indices, parts UNWIND indices AS i WITH \"/\" + reduce(s = \"\", p IN parts[0..i+1] | CASE WHEN s = \"\" THEN p ELSE s + \"/\" + p END) AS folderPath, CASE WHEN i = 0 THEN \"/\" ELSE \"/\" + reduce(s = \"\", p IN parts[0..i] | CASE WHEN s = \"\" THEN p ELSE s + \"/\" + p END) END AS parentPath MERGE (f {{name: folderPath}}) ON CREATE SET f:Passive, f.primaryKey=randomUUID() MERGE (parent {{name: parentPath}}) ON CREATE SET parent:Passive, f.primaryKey=randomUUID() MERGE (parent)-[:namespace]->(f) ",
                        "parameters": {{
                            "name": "{}"
                            }}
                        }}
                    ]
            }}
        )", fullName);
    }

    std::string getPayloadCreateProcessAndLinkPassiveHelpers(std::string pids) {
        return fmt::format(R"(
            {{
                "statements":
                    [
                        {{ "statement": "UNWIND $pids AS nodeData MERGE (n {{pid: nodeData.pid}}) ON CREATE SET n:Active, n.primaryKey=randomUUID(), n.name = nodeData.name WITH collect(nodeData.pid) AS pids UNWIND range(0, size(pids)-2) AS idx WITH pids[idx] AS fromPid, pids[idx + 1] AS toPid WHERE fromPid <> toPid MATCH (from {{pid: fromPid}}) MATCH (to {{pid: toPid}}) MERGE (to)-[:process]->(from) WITH collect(from) AS nodesFrom, collect(to) AS nodesTo UNWIND nodesFrom + nodesTo AS n RETURN DISTINCT {{pid: toInteger(n.pid), id: n.primaryKey}} AS result ",
                        "parameters": {{
                            "pids": {}
                        }}
                        }}
                    ]
            }}
        )", pids);
    }
    
    std::string getPayloadCreateProcessAndUpdatePassiveHelpers(std::string fullName, std::string pids) {
        return fmt::format(R"(
            {{
                "statements":
                    [
                        {{ "statement": "MATCH (targetNode:Active {{name: $name}}) WITH targetNode, $pids AS values SET targetNode.pid = values[0] WITH targetNode, values UNWIND range(0, size(values)-2) AS i MATCH (targetNode:Active {{pid: values[i]}}) OPTIONAL MATCH (parent)-[:process]->(targetNode) SET parent.pid = values[i+1] WITH parent, targetNode, values, i UNWIND [targetNode, parent] AS n RETURN DISTINCT {{ pid: toInteger(n.pid), id: n.primaryKey }} AS result ",
                        "parameters": {{
                            "name": "{}",
                            "pids": {}
                            }}
                        }}
                    ]
            }}
        )", fullName, pids);
    }

    std::string createRoot(std::string hostName, std::string macAdress) {
        return fmt::format(R"(
            {{
                "statements":
                    [
                        {{ "statement": "MERGE (n {{pid: 0}}) ON CREATE SET n.primaryKey=randomUUID() SET n:Passive, n.name = $name, n.mac = $mac RETURN DISTINCT {{ id: n.primaryKey }} ",
                        "parameters": {{
                            "name": "{}",
                            "mac": "{}"
                            }}
                        }}
                    ]
            }}
        )", hostName, macAdress);
    }
}
