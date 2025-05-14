#include <neo4j/node/node.hpp>

#include <string>
#include <fmt/core.h>


namespace node {

    std::string getPayload(std::string name, u_int64_t handle, pid_t pid) {
        return fmt::format(R"(
            {{ 
                "statements":
                    [
                        {{ "statement": "MERGE (n:Node{{name:$name}}) ON CREATE SET n.handle=$handle, n.state=1, n.stateChangeTime = TIMESTAMP(), n.pid=$pid, n.bootcounter = 1 ON MATCH SET n.handle=$handle, n.state=1, n.stateChangeTime = TIMESTAMP(), n.pid=$pid, n.bootcounter = n.bootcounter+1, n.Services=[], n.Clients=[], n.ActionServices=[], n.ActionClients=[] WITH n OPTIONAL MATCH (n)-[r]-() WHERE TYPE(r) IN ['publishes_to', 'subscription', 'client_for', 'action_for'] DELETE r WITH n RETURN DISTINCT n ",
                        "parameters": {{
                            "name": "{}",
                            "handle": "{}",
                            "pid": "{}"
                            }}
                        }}
                    ]
            }}
        )", name, handle, pid);
    }

    std::string getPayloadRequestByPrimaryKey(pid_t pid) {
        return fmt::format(R"(
            {{ nodeRequest
                "statements":
                    [
                        {{ "statement": "MATCH (n) WITH n, toInteger(last(SPLIT(elementId(n), \":\"))) AS extractedId WHERE extractedId = $primaryKey OPTIONAL MATCH (n)-[r]->(outgoingNeighbor) WITH n, elementId(n) AS fullElementId, extractedId, toInteger(n.pid) AS pid, type(r) AS outgoingRelType, collect({{id: toInteger(last(SPLIT(elementId(outgoingNeighbor), \":\"))), name: r.name, direction: 'outgoing'}}) AS outgoingNeighbors OPTIONAL MATCH (n)<-[r2]-(incomingNeighbor) WITH n, fullElementId, extractedId, pid, outgoingNeighbors, outgoingRelType, type(r2) AS incomingRelType, collect({{id: toInteger(last(SPLIT(elementId(incomingNeighbor), \":\"))), name: r2.name, direction: 'incoming'}}) AS incomingNeighbors RETURN DISTINCT n {{.*, pid: pid}}, collect(DISTINCT {{relationship: outgoingRelType, nodes: outgoingNeighbors}}) AS outgoing, collect(DISTINCT {{relationship: incomingRelType, nodes: incomingNeighbors}}) AS incoming ",
                        "parameters": {{
                            "primaryKey": {}
                            }}
                        }}
                    ]
            }}
        )", pid);
    }

    std::string getPayloadSetNodeOffline(pid_t pid) {
        return fmt::format(R"(
        {{
            "statements":
                [
                    {{ "statement": "MATCH (n:Node {{pid:$pid}}) SET n.state = 0, n.stateChangeTime = TIMESTAMP() ",
                    "parameters": {{
                        "pid": "{}"
                        }}
                    }}
                ]
        }}
        )", pid);
    }

}