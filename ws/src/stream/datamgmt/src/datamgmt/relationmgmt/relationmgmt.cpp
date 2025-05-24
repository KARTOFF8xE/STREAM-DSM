#include <bits/stdc++.h>
#include <iostream>
#include <sstream>
#include <map>
#include <unistd.h>
#include <limits.h>
#include <chrono>
using namespace std::chrono_literals;

#include <nlohmann/json.hpp>

#include <ipc/ipc-client.hpp>
#include <neo4j/roots/roots.hpp>
#include <curl/myCurl.hpp>

#include "datamgmt/utils.hpp"
#include "datamgmt/relationmgmt/relationmgmt.hpp"
#include "pipe/pipe.hpp"


struct ProcessData {
    pid_t       pid = 0;
    pid_t       ppid;
    std::string name;
    bool        exists = false;
};

ProcessData getProcessDatabyPID(pid_t pid) {
    ProcessData pd { .pid = pid };

    std::ifstream fp(
        std::string("/proc/") +
        std::to_string(pid) + 
        std::string("/stat")
    );

    if (!fp.is_open()) return pd;
    pd.exists = true;

    std::string payload;
    std::getline(fp, payload);

    int i   = payload.find(" (");
    payload = payload.substr(i + 2 * sizeof(char));
    i       = payload.find(") ");
    pd.name = payload.substr(0, i);
    payload = payload.substr(i + 2 * sizeof(char));
    
    sscanf(payload.c_str(), "%*c %d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d",
        &pd.ppid);

    fp.close();
    return pd;
}

void reduceProcessData(std::vector<ProcessData> &pdv) {
    for (size_t i = pdv.size(); i-- > 0;) {
        if (pdv.at(i).name == "ros2") break;

        pdv.pop_back();
    }

    pdv.push_back(ProcessData{ .name = getHostname() });
}

std::string getParameterString(std::vector<ProcessData> pdv) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < pdv.size(); i++) {
        oss << "{\"pid\": "
            << pdv.at(i).pid
            << ", \"name\": \""
            << pdv.at(i).name
            << "\""
            << "}";

        if (i < pdv.size() - 1) oss << ", ";
    }
    oss << "]";
    return oss.str();
}

std::string getParameterString2(std::vector<ProcessData> pdv) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < pdv.size(); i++) {
        oss << "\""
            << pdv.at(i).pid
            << "\"";

        if (i < pdv.size() - 1) oss << ", ";
    }
    oss << "]";
    return oss.str();
}

struct Pair {
    pid_t           pid;
    primaryKey_t    primaryKey;
};

std::vector<Pair> extractPIDandID(const std::string& payload) {
    nlohmann::json j = nlohmann::json::parse(payload);

    std::vector<Pair> result;

    for (const auto& item : j["results"][0]["data"]) {
        if (item.contains("row") && !item["row"].empty()) {           
            result.push_back(
                Pair {
                    .pid        = item["row"][0]["pid"].get<pid_t>(),
                    .primaryKey = item["row"][0]["id"].get<primaryKey_t>()
                }
            );
        }
    }
    
    return result;
}



namespace relationMgmt {

void relationMgmt(std::map<Module_t, pipe_ns::Pipe> pipes, std::atomic<bool> &running) {
    std::cout << "started relationMgmt" << std::endl;
    int pipeToProcessobserver_w = pipes[PROCESSOBSERVER].write;
    
    auto then = std::chrono::steady_clock::now();
    while (true) {

        ssize_t ret = -1;
        pipe_ns::UnionResponse unionResponse;
        pipe_ns::MsgType type;
        ret = pipe_ns::readT<pipe_ns::UnionResponse>(pipes[NODEANDTOPICOBSERVER].read, unionResponse, &type);
        if (ret == -1) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now-then);
            auto sleepTime = 1s - elapsed;
            if (sleepTime > 0s) {
                std::this_thread::sleep_for(sleepTime);
            }
            then = std::chrono::steady_clock::now();

            continue;
        }

        if (type == pipe_ns::MsgType::TOPICRESPONSE) {
            {
            /*** Namespace-Tree ***/
            curl::push(
                createRoot::getPayloadCreateNameSpaceAndLinkPassiveHelpers(unionResponse.topicResp.name),
                curl::NEO4J
            );
            }

            continue;
        }

        {
            /*** Namespace-Tree ***/
            curl::push(
                createRoot::getPayloadCreateNameSpaceAndLinkPassiveHelpers(unionResponse.nodeResp.name),
                curl::NEO4J
            );
        }
        {
            /*** Process-Tree ***/
            std::vector<ProcessData> pdv;
                pid_t pid = unionResponse.nodeResp.pid;
                while (true) {
                    ProcessData pd = getProcessDatabyPID(pid);
                    if (pd.pid == 0) break;

                    pdv.push_back(pd);
                    pid = pd.ppid;
                };
                reduceProcessData(pdv);

            std::string queryResp;
            if (unionResponse.nodeResp.bootCount == 1) {
                queryResp = curl::push(
                    createRoot::getPayloadCreateProcessAndLinkPassiveHelpers(
                        getParameterString(pdv)
                    ),
                    curl::NEO4J
                );
            } else {
                queryResp = curl::push(
                    createRoot::getPayloadCreateProcessAndUpdatePassiveHelpers(
                        unionResponse.nodeResp.name,
                        getParameterString2(pdv)
                    ),
                    curl::NEO4J
                );
            }

            std::vector<Pair> pairs = extractPIDandID(queryResp);

            for (auto pair : pairs) {
                if (pair.pid == 0) continue;
                pipe_ns::writeT<NodeResponse>(
                    pipeToProcessobserver_w,
                    NodeResponse{
                        .primaryKey = pair.primaryKey,
                        .pid        = pair.pid,
                    }
                );
            }
        }
    }

    running.store(false);
}

} 