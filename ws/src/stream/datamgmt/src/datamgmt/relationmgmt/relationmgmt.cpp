#include <bits/stdc++.h>
#include <iostream>
#include <sstream>
#include <map>

#include <ipc/ipc-client.hpp>
#include <neo4j/roots/roots.hpp>
#include <curl/myCurl.hpp>

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

    pdv.push_back(ProcessData{ .name = "root" });
}

std::string getParameterString(std::vector<ProcessData> pdv) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < pdv.size(); i++) {
        oss << "{\"pid\": \""
            << pdv.at(i).pid
            << "\", \"name\": \""
            << pdv.at(i).name
            << "\""
            << "}";

        if (i < pdv.size() - 1) oss << ", ";
    }
    oss << "]";
    return oss.str();
}

namespace relationMgmt {

void relationMgmt(NodeResponse response) {
    {
        /*** Namespace-Tree ***/
        curl::push(
            createRoot::getPayloadCreateNameSpaceAndLinkPassiveHelpers(response.name),
            NEO4J
        );
    }
    {
        /*** Process-Tree ***/
        std::vector<ProcessData> pdv;
        if (response.bootCount == 1) {
            pid_t pid = response.pid;
            while (true) {
                ProcessData pd = getProcessDatabyPID(pid);
                if (pd.pid == 0) break;

                pdv.push_back(pd);
                pid = pd.ppid;
            };

            reduceProcessData(pdv);

            curl::push(
                createRoot::getPayloadCreateProcessAndLinkPassiveHelpers(
                    getParameterString(pdv)
                ),
                NEO4J
            );
        }
    }
}

} 