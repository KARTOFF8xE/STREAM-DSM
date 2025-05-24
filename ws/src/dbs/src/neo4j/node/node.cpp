#include <neo4j/node/node.hpp>

#include <string>
#include <fmt/core.h>


namespace node {

    std::string getPayload(std::string name, u_int64_t handle, pid_t pid, time_t timestamp) {
        return fmt::format(R"(
            {{ 
                "statements":
                    [
                        {{ "statement": "MERGE (n:Active{{name:$name}}) ON CREATE SET n.handle=$handle, n.state=1, n.stateChangeTime = $timestamp, n.pid=$pid, n.bootcounter = 1 ON MATCH SET n.handle=$handle, n.state=1, n.stateChangeTime = TIMESTAMP(), n.pid=$pid, n.bootcounter = n.bootcounter+1, n.Services=[], n.Clients=[], n.ActionServices=[], n.ActionClients=[] WITH n OPTIONAL MATCH (n)-[r]-() WHERE TYPE(r) IN ['publishes_to', 'subscription', 'service_for', 'action_for', 'timer'] DELETE r WITH n RETURN DISTINCT n ",
                        "parameters": {{
                            "name": "{}",
                            "handle": {},
                            "pid": {},
                            "timestamp": {}
                            }}
                        }}
                    ]
            }}
        )", name, handle, pid, timestamp);
    }

    std::string getPayloadRequestByPrimaryKey(pid_t pid) {
        return fmt::format(R"(
            {{
                "statements":
                    [
                        {{ "statement": "MATCH (n) WITH n, toInteger(last(SPLIT(elementId(n), \":\"))) AS extractedId WHERE extractedId = $primaryKey OPTIONAL MATCH (n)-[r]->(outgoingNeighbor) WITH n, elementId(n) AS fullElementId, extractedId, toInteger(n.pid) AS pid, type(r) AS outgoingRelType, collect({{id: toInteger(last(SPLIT(elementId(outgoingNeighbor), \":\"))), name: r.name, direction: 'outgoing', frequency: r.frequency}}) AS outgoingNeighbors OPTIONAL MATCH (n)<-[r2]-(incomingNeighbor) WITH n, fullElementId, extractedId, pid, outgoingNeighbors, outgoingRelType, type(r2) AS incomingRelType, collect({{id: toInteger(last(SPLIT(elementId(incomingNeighbor), \":\"))), name: r2.name, direction: 'incoming', frequency: r2.frequency}}) AS incomingNeighbors RETURN DISTINCT n {{.*, pid: pid}}, collect(DISTINCT {{relationship: outgoingRelType, nodes: outgoingNeighbors}}) AS outgoing, collect(DISTINCT {{relationship: incomingRelType, nodes: incomingNeighbors}}) AS incoming  ",
                        "parameters": {{
                            "primaryKey": {}
                            }}
                        }}
                    ]
            }}
        )", pid);
    }

    std::string getPayloadSetNodeOffline(pid_t pid, time_t timestamp) {
        return fmt::format(R"(
        {{
            "statements":
                [
                    {{ "statement": "MATCH (n:Active {{pid:$pid}}) SET n.state = 0, n.stateChangeTime = $timestamp ",
                    "parameters": {{
                        "pid": {},
                        "timestamp": {}
                        }}
                    }}
                ]
        }}
        )", pid, timestamp);
    }

    std::string getPayloadSetStateMachine(u_int64_t handle, u_int64_t stateMachine, u_int64_t state) {
        return fmt::format(R"(
        {{
            "statements":
                [
                    {{ "statement": "MATCH (n:Active {{handle: $handle}}) set n.stateMachine= $statemachine, n.state= $state ",
                    "parameters": {{
                        "handle": {},
                        "statemachine": {},
                        "state": {}
                        }}
                    }}
                ]
        }}
        )", handle, stateMachine, state);       
    }

    std::string getPayloadSetStateTransition(u_int64_t stateMachine, sharedMem::State state, time_t timestamp) {
        return fmt::format(R"(
        {{
            "statements":
                [
                    {{ "statement": "MATCH (n:Active {{stateMachine: $statemachine}}) set n.state = $state, n.stateChangeTime = $timestamp RETURN toInteger(last(SPLIT(elementId(n), \":\"))) ",
                    "parameters": {{
                        "statemachine": {},
                        "state": {},
                        "timestamp": {}
                        }}
                    }}
                ]
        }}
        )", stateMachine, state, timestamp);       
    }

}