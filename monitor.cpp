#include <iostream>
#include <vector>
#include <dirent.h>
#include <string.h>
#include <unistd.h>
#include <bits/stdc++.h>

struct ProcessData{
    int pid;
    int ppid;
    std::string exe_filename;
    std::string process_state;
    long usr_cpu_clocks;
    long krnl_cpu_clocks;
    long logged_clock_time;
    double cpu_utilisation;
    int process_priority;
    long v_mem_size;
    int last_cpu;
};
typedef struct _IO_FILE FILE;

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

ProcessData getProcDataByPID(int pid) {
    std::ifstream fp(
        std::string("/proc/") +
        std::to_string(pid) + 
        std::string("/stat")
    );
    std::string payload;
    std::getline(fp, payload);

    ProcessData pd {
        .logged_clock_time = get_cpu_time(),
    };

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
    return pd;
}

// Strip the filename to 10 characters with + at the end
const char* format_filename(const char *exe_filename) {
    int exe_filename_length, max_filename_length;
    char *new_filename;
    exe_filename_length=strlen(exe_filename);
    max_filename_length=10;
    if (exe_filename_length<=max_filename_length) return exe_filename;
    new_filename = (char*)malloc(max_filename_length+1);
    for (int i=0; i<max_filename_length-1; i++) { new_filename[i]=exe_filename[i];}
    new_filename[max_filename_length-1]='+';
    new_filename[max_filename_length]='\0';
    return new_filename;
}

// Display the top num_processes statistics
void display_process_data(const ProcessData &pd) {
    system("clear");
    printf("%-15s%-10s%-15s%-15s%-10s%-10s\n", 
        ".EXE NAME", "PID", "PRIORITY", "VRAM (MB)", "CPU", "% CPU");
    printf("%-15s%-10d%-15d%-15ld%-10d%-10.2f\n",
        format_filename(pd.exe_filename.c_str()), pd.pid, pd.process_priority, pd.v_mem_size, pd.last_cpu, pd.cpu_utilisation);
}

// Get the cpu delta time
long get_cpu_delta_time(ProcessData *pd) {
    long logged_cpu_time, current_cpu_time, delta_cpu_time;
    logged_cpu_time=pd->logged_clock_time; // Get the logged cpu time of the process
    current_cpu_time=get_cpu_time(); // Get the current cpu time
    delta_cpu_time=current_cpu_time-logged_cpu_time; // Find the delta time
    return delta_cpu_time;
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
long get_process_delta_time(ProcessData *pd) {
    long previous_process_time, current_process_time, delta_process_time;
    previous_process_time   = pd->usr_cpu_clocks + pd->krnl_cpu_clocks; // Find logged clock time of process
    current_process_time    = get_process_cpu_time(pd->pid); // Get the new clock time of the process
    delta_process_time      = current_process_time - previous_process_time; // Find the delta process time
    return delta_process_time;
}

// Calculate the cpu utilisation of each process
void calculate_cpu_utilisation(ProcessData &pd) {
    double cpu_utilisation   = (double)get_process_delta_time(&pd)/get_cpu_delta_time(&pd);
    pd.cpu_utilisation      = 100 * cpu_utilisation;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "need PID as argument" << std::endl;
        return 1;
    }
    int pid = std::stoi(argv[1]);
    std::cout << "pid: " << pid << std::endl;

    // char buffer[1024];
    ProcessData process_data;

    while (true) {
        process_data = getProcDataByPID(pid);

        sleep(1);
        calculate_cpu_utilisation(process_data);
        display_process_data(process_data);
    }
}