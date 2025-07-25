#include <neo4j/node/node.hpp>

#include <string>
#include <fmt/core.h>


namespace node {

    std::string getPayload(std::string name, u_int64_t handle, sharedMem::State state, pid_t pid, time_t timestamp) {
        return fmt::format(R"(
            {{ 
                "statements":
                    [
                        {{ "statement": "MERGE (n:Active{{name:$name}}) ON CREATE SET n.primaryKey=randomUUID(), n.handle=$handle, n.state=$state, n.stateChangeTime = $timestamp, n.pid=$pid, n.bootcounter = 1 ON MATCH SET n.handle=$handle, n.state=$state, n.stateChangeTime = TIMESTAMP(), n.pid=$pid, n.bootcounter = n.bootcounter+1, n.Services=[], n.Clients=[], n.ActionServices=[], n.ActionClients=[] WITH n OPTIONAL MATCH (n)-[r]-() SET r.active=false WITH n RETURN DISTINCT n ",
                        "parameters": {{
                            "name": "{}",
                            "handle": {},
                            "state": {},
                            "pid": {},
                            "timestamp": {}
                            }}
                        }}
                    ]
            }}
        )", name, handle, state, pid, timestamp);
    }

    std::string getPayloadSearch(std::string name) {
        return fmt::format(R"(
            {{ 
                "statements":
                    [
                        {{ "statement": "MATCH (n:Active {{name: $name}}) RETURN n.primaryKey ",
                        "parameters": {{
                            "name": "{}"
                            }}
                        }}
                    ]
            }}
        )", name);
    }


    std::string getPayloadRequestByPrimaryKey(std::string primaryKey) {
        return fmt::format(R"(
            {{
                "statements":
                    [
                        {{ "statement": "MATCH (n) WITH n, n.primaryKey AS extractedId WHERE extractedId = $primaryKey OPTIONAL MATCH (n)-[r]->(outgoingNeighbor) WITH n, elementId(n) AS fullElementId, extractedId, toInteger(n.pid) AS pid, type(r) AS outgoingRelType, collect({{id: outgoingNeighbor.primaryKey, edge_id: r.primaryKey, serviceName: r.serviceName, direction: 'outgoing', frequency: r.frequency, actionName: r.actionName, mode: r.mode}}) AS outgoingNeighbors OPTIONAL MATCH (n)<-[r2]-(incomingNeighbor) WITH n, fullElementId, extractedId, pid, outgoingNeighbors, outgoingRelType, type(r2) AS incomingRelType, collect({{id: incomingNeighbor.primaryKey, edge_id: r2.primaryKey, serviceName: r2.serviceName, direction: 'incoming', frequency: r2.frequency, actionName: r2.actionName, mode: r2.mode}}) AS incomingNeighbors RETURN DISTINCT n {{.*, pid: pid}}, collect(DISTINCT {{relationship: outgoingRelType, nodes: outgoingNeighbors}}) AS outgoing, collect(DISTINCT {{relationship: incomingRelType, nodes: incomingNeighbors}}) AS incoming ",
                        "parameters": {{
                            "primaryKey": "{}"
                            }}
                        }}
                    ]
            }}
        )", primaryKey);
    }

    std::string getPayloadSetNodeStateByPID(pid_t pid, time_t timestamp, sharedMem::State state) {
        return fmt::format(R"(
        {{
            "statements":
                [
                    {{ "statement": "MATCH (n:Active {{pid:$pid}}) SET n.state = $state, n.stateChangeTime = $timestamp WITH n OPTIONAL MATCH (n)-[e]->() SET e.active=$edgeActive WITH n OPTIONAL MATCH (n)<-[e:subscribing]-() SET e.active=$edgeActive RETURN DISTINCT {{ stateChangeTime: n.stateChangeTime, state: n.state, primaryKey: n.primaryKey }} ",
                    "parameters": {{
                        "pid": {},
                        "timestamp": {},
                        "state": {},
                        "edgeActive": {}
                        }}
                    }}
                ]
        }}
        )", pid, timestamp, state, state==sharedMem::State::ACTIVE);
    }

    std::string getPayloadSetStateMachine(u_int64_t handle, u_int64_t stateMachine, sharedMem::State state) {
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
                    {{ "statement": "MATCH (n:Active {{stateMachine: $statemachine}}) SET n.state = $state, n.stateChangeTime = $timestamp WITH n MATCH (n)-[e]->() SET e.active=$edgeActive WITH n MATCH (n)<-[e:subscribing]-() SET e.active=$edgeActive RETURN n.primaryKey ",
                    "parameters": {{
                        "statemachine": {},
                        "state": {},
                        "timestamp": {},
                        "edgeActive": {}
                        }}
                    }}
                ]
        }}
        )", stateMachine, state, timestamp, state==sharedMem::State::ACTIVE);       
    }

    std::string getAdjacentIncomingEdges(std::string primaryKey) {
        return fmt::format(R"(
        {{
            "statements":
                [
                    {{ "statement": "MATCH ()-[edge]->(n {{primaryKey: $primaryKey}}) WHERE edge.primaryKey IS NOT NULL RETURN collect(DISTINCT edge.primaryKey) ",
                    "parameters": {{
                        "primaryKey": "{}"
                        }}
                    }}
                ]
        }}
        )", primaryKey);       
    }


    std::string getAdjacentOutgoingEdges(std::string primaryKey) {
        return fmt::format(R"(
        {{
            "statements":
                [
                    {{ "statement": "MATCH (n {{primaryKey: $primaryKey}})-[edge]->() WHERE edge.primaryKey IS NOT NULL RETURN collect(DISTINCT edge.primaryKey) ",
                    "parameters": {{
                        "primaryKey": "{}"
                        }}
                    }}
                ]
        }}
        )", primaryKey);       
    }

}