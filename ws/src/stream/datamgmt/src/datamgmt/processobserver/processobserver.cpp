#include <bits/stdc++.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include <limits.h>
#include <sys/ioctl.h>
#include <net/if.h> 
#include <netinet/in.h>
#include <string.h>
#include <dirent.h>
#include <unistd.h>
#include <chrono>
using namespace std::chrono_literals;

#include <nlohmann/json.hpp>

#include <ipc/ipc-client.hpp>
#include <neo4j/roots/roots.hpp>
#include <influxdb/influxdb.hpp>
#include <curl/myCurl.hpp>

#include "datamgmt/utils.hpp"
#include "datamgmt/relationmgmt/relationmgmt.hpp"
#include "pipe/pipe.hpp"


struct FullProcessData {
    primaryKey_t    primaryKey;
    pid_t           pid;
    pid_t           ppid;
    std::string     exe_filename;
    std::string     process_state;
    long            usr_cpu_clocks;
    long            krnl_cpu_clocks;
    long            logged_clock_time;
    double          cpu_utilisation;
    int             process_priority;
    long            v_mem_size;
    int             last_cpu;
};
typedef struct _IO_FILE FILE;


struct CPUData;
void readCPUUtil(CPUData &cd);

struct CPUData {
    primaryKey_t    primaryKey;

    long            totalJiffiesThen;
    long            totalJiffiesNow;
    long            workJiffiesThen;
    long            workJiffiesNow;
    double          utilization;

    CPUData() {
        readCPUUtil(*this);
    };
};

// Open the pid stats file
FILE* open_pid_stats(int pid) {
    char cmd_line_path[256];
    FILE *fp;
    sprintf(cmd_line_path, "/proc/%d/stat", pid);
    fp=fopen(cmd_line_path, "r");
    if (!fp) { 
        char err_buffer[300]; 
        sprintf(err_buffer, "Cannot open %s\n", cmd_line_path);
        return NULL;
    }
    return fp;
}

// Read the proc stat to get total cpu time
long get_cpu_time() {
    char cmd_line_path[256], buffer[512], *token;
    long total;
    int entry;
    FILE *fp;
    sprintf(cmd_line_path, "/proc/stat");
    fp=fopen(cmd_line_path, "r");
    fgets(buffer, 512, fp);
    total=0;
    token = strtok(buffer, " ");
    entry = 0;
    while (token!=NULL) { 
        if (entry>0)
            total+=atol(token);
        token=strtok(NULL, " ");
        entry++;
    }
    fclose(fp);
    return total;
}

void getProcDataByPID(int pid, FullProcessData &pd) {
    std::ifstream fp(
        std::string("/proc/") +
        std::to_string(pid) + 
        std::string("/stat")
    );
    std::string payload;
    std::getline(fp, payload);

    pd.logged_clock_time = get_cpu_time();

    int i = payload.find(" (");
    pd.pid = std::atoi(payload.substr(0, i).c_str());
    payload = payload.substr(i + 2 * sizeof(char));
    i = payload.find(") ");
    pd.exe_filename = payload.substr(0, i);
    payload = payload.substr(i + 2*sizeof(char));
    
    sscanf(payload.c_str(), "%*c %d %*d %*d %*d %*d %*d %*d %*d %*d %*d %ld %ld %*d %*d %*d %*d %*d %*d %*d %ld %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %d",
        &pd.ppid, &pd.usr_cpu_clocks, &pd.krnl_cpu_clocks, &pd.v_mem_size, &pd.last_cpu);

    pd.v_mem_size = pd.v_mem_size / 1048576;

    fp.close();
}

// Strip the filename to 10 characters with + at the end
const char* format_filename(const char *exe_filename) {
    int max_filename_length = 10;
    char *new_filename;
    if (strlen(exe_filename)<=max_filename_length) return exe_filename;
    new_filename = (char*)malloc(max_filename_length+1);
    for (int i=0; i<max_filename_length-1; i++) { new_filename[i]=exe_filename[i];}
    new_filename[max_filename_length-1]='+';
    new_filename[max_filename_length]='\0';

    return new_filename;
}

// Get the cpu delta time
long get_cpu_delta_time(FullProcessData *pd) {
    return get_cpu_time() - pd->logged_clock_time;
}

long get_process_cpu_time(int pid) {
    FILE* fp;
    char buffer[512], *token;
    long total;
    int entry;
    if ((fp=open_pid_stats(pid))==NULL) { return -1; }
    fgets(buffer, 512, fp);
    token=strtok(buffer, " ");
    total=entry=0;
    while (token!=NULL) { 
        if (entry==13||entry==14) { total+=atol(token);}
        token=strtok(NULL, " "); 
        entry++;
    }
    fclose(fp);
    return total;
}

// Get the process delta time
long get_process_delta_time(FullProcessData *pd) {
    return get_process_cpu_time(pd->pid) - (pd->usr_cpu_clocks + pd->krnl_cpu_clocks); // Find the delta process time
}

// Calculate the cpu utilisation of each process
double get_cpu_utilisation(FullProcessData &pd) {
    return (double)get_process_delta_time(&pd)/get_cpu_delta_time(&pd);;
}

void readCPUUtil(CPUData &cd) {
    std::ifstream fp(
        std::string("/proc/stat")
    );
    std::string payload;
    std::getline(fp, payload);

    long user, nice, system, idle, iowait, irq, softirq;
    sscanf(payload.c_str(), "%*s %ld %ld %ld %ld %ld %ld %ld",
    &user, &nice, &system, &idle, &iowait, &irq, &softirq);


    cd.workJiffiesNow = user + nice + system;
    cd.totalJiffiesNow = cd.workJiffiesNow + idle + iowait + irq + softirq;
}

void calcCPUUtil(CPUData &cd) {
    cd.totalJiffiesThen = cd.totalJiffiesNow;
    cd.workJiffiesThen = cd.workJiffiesNow;

    readCPUUtil(cd);

    cd.utilization = (double)(cd.workJiffiesNow - cd.workJiffiesThen) / (double)(cd.totalJiffiesNow - cd.totalJiffiesThen);
}

std::string getMacAddress() {
    struct ifreq ifr;
    struct ifconf ifc;
    char buf[1024];
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock == -1) return "";

    ifc.ifc_len = sizeof(buf);
    ifc.ifc_buf = buf;
    if (ioctl(sock, SIOCGIFCONF, &ifc) == -1) { close(sock); return ""; }

    struct ifreq* it = ifc.ifc_req;
    const struct ifreq* const end = it + (ifc.ifc_len / sizeof(struct ifreq));
    for (; it != end; ++it) {
        strcpy(ifr.ifr_name, it->ifr_name);
        if (ioctl(sock, SIOCGIFFLAGS, &ifr) == 0 && !(ifr.ifr_flags & IFF_LOOPBACK)) {
            if (ioctl(sock, SIOCGIFHWADDR, &ifr) == 0) {
                close(sock);
                unsigned char* mac = (unsigned char*)ifr.ifr_hwaddr.sa_data;
                std::ostringstream oss;
                for (int i = 0; i < 6; ++i) {
                    oss << (i ? ":" : "") << std::hex << std::uppercase << (int)mac[i];
                }
                return oss.str();
            }
        }
    }
    close(sock);
    
    return "";
}

namespace processObserver {

void processObserver(std::map<Module_t, Pipe> pipes, std::atomic<bool> &running) {
    std::cout << "started processObserver" << std::endl;
    
    std::vector<FullProcessData> processVec;

    CPUData cpuData;
    {
        std::string payload = curl::push(
            createRoot::createRoot(getHostname(), getMacAddress()),
            curl::NEO4J
        );
        nlohmann::json j = nlohmann::json::parse(payload);

        if (j.contains("results") && !j["results"].empty() &&
            j["results"][0].contains("data") && !j["results"][0]["data"].empty() &&
            j["results"][0]["data"][0].contains("row") && !j["results"][0]["data"][0]["row"].empty() &&
            j["results"][0]["data"][0]["row"][0].contains("id"))
            {
            cpuData.primaryKey = j["results"][0]["data"][0]["row"][0]["id"];
        } else {
            std::cerr << "Error: Failed to find primaryKey ('id') in the expected structure." << std::endl;
        }
    }


    auto then = std::chrono::steady_clock::now();
    while (true) {
        std::vector<influxDB::ValuePairs> pairs;
        for (auto it = processVec.begin(); it != processVec.end(); ++it) {
            int index = std::distance(processVec.begin(), it);

            processVec.at(index).cpu_utilisation = get_cpu_utilisation(processVec.at(index));
            if (processVec.at(index).cpu_utilisation < 0) {
                std::cout << "removed Process " << processVec.at(index).exe_filename << " with pid " << processVec.at(index).pid << std::endl;
                processVec.erase(it);
                if (processVec.size() == 0) break;
                it--;
                continue;
            }

            pairs.push_back(influxDB::ValuePairs{
                .primaryKey = processVec.at(index).primaryKey,
                .value      = processVec.at(index).cpu_utilisation
            });

            getProcDataByPID(processVec.at(index).pid, processVec.at(index));

        }
        curl::push(
            influxDB::createPayloadMultipleValSameTime(influxDB::CPU_UTILIZATION, pairs),
            curl::INFLUXDB_WRITE
        );
        
        calcCPUUtil(cpuData);
        curl::push(
            influxDB::createPayloadSingleVal(influxDB::CPU_UTILIZATION, cpuData.primaryKey, cpuData.utilization),
            curl::INFLUXDB_WRITE
        );

        NodeResponse response;
        ssize_t ret = -1;
        ret = readT<NodeResponse>(pipes[RELATIONMGMT].read, response);
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

        if (response.pid == 0) {
            cpuData.primaryKey = response.primaryKey;
            continue;
        }

        std::cout << "Added process with pid: " << response.pid << " and primaryKey: " << response.primaryKey << std::endl;
        processVec.push_back(FullProcessData{
            .primaryKey = response.primaryKey,
            .pid        = response.pid
        });
    }

    running.store(false);
}

} 