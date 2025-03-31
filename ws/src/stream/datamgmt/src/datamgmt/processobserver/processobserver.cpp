#include <bits/stdc++.h>
#include <iostream>
#include <sstream>
#include <map>
#include <chrono>

#include <ipc/ipc-client.hpp>
#include <neo4j/roots/roots.hpp>
#include <curl/myCurl.hpp>

#include "datamgmt/relationmgmt/relationmgmt.hpp"
#include "pipe/pipe.hpp"


namespace processObserver {

void processObserver(std::map<Module_t, Pipe> pipes, std::atomic<bool> &running) {
    std::cout << "started processObserver" << std::endl;
    
    auto then = std::chrono::steady_clock::now();
    while (true) {

        NodeResponse response;
        ssize_t ret = -1;
        ret = readT<NodeResponse>(pipes[RELATIONMGMT].read, response);
        if (ret == -1) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now-then).count();
            auto sleepTime = 1000000 - elapsed;
            if (sleepTime > 0) {
                std::this_thread::sleep_for(std::chrono::microseconds(sleepTime));
            }
            then = std::chrono::steady_clock::now();

            continue;
        }
        std::cout << response.pid << std::endl;

    }

    running.store(false);
}

} 