#include <bits/stdc++.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include <chrono>
using namespace std::chrono_literals;

#include <dirent.h>
#include <string.h>
#include <unistd.h>

#include <ipc/ipc-client.hpp>
#include <neo4j/roots/roots.hpp>
#include <influxdb/influxdb.hpp>
#include <curl/myCurl.hpp>

#include "datamgmt/relationmgmt/relationmgmt.hpp"
#include "pipe/pipe.hpp"


struct ExtendedProcessData{
    primaryKey_t    primaryKey;
    pid_t           pid;
    pid_t           ppid;
    std::string     exe_filename;
    std::string     process_state;
    long            usr_cpu_clocks;
    long            krnl_cpu_clocks;
    long            logged_clock_time;
    float           cpu_utilisation;
    int             process_priority;
    long            v_mem_size;
    int             last_cpu;
};
typedef struct _IO_FILE FILE;


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

long get_process_delta_time(ExtendedProcessData *pd) {
    long previous_process_time, current_process_time, delta_process_time;
    previous_process_time   = pd->usr_cpu_clocks + pd->krnl_cpu_clocks; // Find logged clock time of process
    current_process_time    = get_process_cpu_time(pd->pid); // Get the new clock time of the process
    delta_process_time      = current_process_time - previous_process_time; // Find the delta process time
    return delta_process_time;
}

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

long get_cpu_delta_time(ExtendedProcessData *pd) {
    long logged_cpu_time, current_cpu_time, delta_cpu_time;
    logged_cpu_time=pd->logged_clock_time; // Get the logged cpu time of the process
    current_cpu_time=get_cpu_time(); // Get the current cpu time
    delta_cpu_time=current_cpu_time-logged_cpu_time; // Find the delta time
    return delta_cpu_time;
}

void calculate_cpu_utilisation(ExtendedProcessData &pd) {
    float cpu_utilisation   = (float)get_process_delta_time(&pd)/get_cpu_delta_time(&pd);
    pd.cpu_utilisation      = 100 * cpu_utilisation;
}

namespace processObserver {

void processObserver(std::map<Module_t, Pipe> pipes, std::atomic<bool> &running) {
    std::cout << "started processObserver" << std::endl;
    
    std::vector<ExtendedProcessData> processVec;

    auto then = std::chrono::steady_clock::now();
    while (true) {

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

        // processVec.push_back(ExtendedProcessData{
        //     .primaryKey = response.primaryKey,
        //     .pid = response.pid
        // });

        // for (auto process : processVec) {
        //     calculate_cpu_utilisation(process);
        // }

        // std::vector<influxDB::ValuePairs> pairs;
        // TODO: hier gehts weiter

    }

    running.store(false);
}

} 