#include <string>
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>
#include <mutex>
#include <condition_variable>

#include <ipc/ipc-server.hpp>

std::mutex m;
std::condition_variable cv;

struct Client {
    pid_t pid;
    requestId_t requestId;
};

void procObserver(IpcServer server, Client client) {
    std::vector<Client> procObserverClients;
    procObserverClients.push_back(client);

    // TODO gather a list with all Node-PIDs (neo4j query)
    // TODO needs a communication with tracer (subscribe Events)

    while (procObserverClients.size() > 0) {
        // TODO receive event based node updates
        // TODO receive information about eventual new clients (or unsubscriptions)
        // TODO check if processes are still alive
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // TODO unsubscribe Events
}

int main() {
    IpcServer server(0);
    bool procObserverRunning = false;

    while (true) {
        // TODO receive different messages as a server (subscriptions and unsubscriptions)
        // TODO handle threads that are running
    }
}