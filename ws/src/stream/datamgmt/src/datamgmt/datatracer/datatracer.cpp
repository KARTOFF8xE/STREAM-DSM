#include "datamgmt/datatracer/datatracer.hpp"

#include <vector>
#include <iostream>
#include <map>
#include <cctype>
using namespace std::chrono_literals;

#include <nlohmann/json.hpp>

#include <ipc/util.hpp>
#include <ipc/sharedMem.hpp>
#include <influxdb/influxdb.hpp>
#include <curl/myCurl.hpp>
#include "datamgmt/utils.hpp"
#include "datamgmt/relationmgmt/relationmgmt.hpp"
#include "pipe/pipe.hpp"


namespace datatracer {

void datatracer(const IpcServer &server,  std::map<Module_t, pipe_ns::Pipe> pipes, std::atomic<bool> &running) {
    std::cout << "started Tracer (Endpoint)" << std::endl;

    std::vector<std::unique_ptr<sharedMem::SHMChannel<sharedMem::InputValue>>> channels;

    auto then = std::chrono::steady_clock::now();
    auto start = std::chrono::high_resolution_clock::now();
    size_t counter = 0;
    std::vector<influxDB::ValuePairs> valuePairs;
    while (true) {
        requestId_t requestID;
        pid_t pid;
        std::optional<SHMAddressRequest> request = server.receiveSHMAddressRequest(requestID, pid, false);
        if (request.has_value()) {
            SHMAddressRequest payload = request.value();

            
            SHMAddressResponse response;
            util::parseString(response.memAddress, sharedMem::parseShmName(pid, requestID));

            channels.push_back(std::make_unique<sharedMem::SHMChannel<sharedMem::InputValue>>(response.memAddress, true));

            server.sendSHMAddressResponse(response, pid, false);
            std::cout << "response.memAddress: " << response.memAddress << std::endl;
            continue;
        }

        for (const auto& channel : channels) {
            sharedMem::InputValue msg;
            if(!channel->receive(msg)) continue;
            influxDB::ValuePairs v {
                    .primaryKey = msg.primaryKey,
                    .timestamp  = msg.timestamp,
                    .value      = msg.value,
            };
            valuePairs.push_back(v);
            switch (msg.type) {
                case sharedMem::MessageType::CPU:
                    valuePairs.back().attribute = influxDB::CPU_UTILIZATION;
                    break;
                case sharedMem::MessageType::DISK:
                    valuePairs.back().attribute = influxDB::DISK_UTILIZATION;
                    break;
                default:
                    break;
            }
            auto now = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = now - start;
            if (++counter >= 100000 || 
                elapsed.count() >= 1.0
            ) {
                counter = 0;
                curl::push(influxDB::createPayloadMultipleValSameTime(valuePairs), curl::INFLUXDB_WRITE);
                valuePairs.clear();
                start = now;
            }
        }
        continue;
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now-then);
        auto sleepTime = 1ms - elapsed;
        if (sleepTime > 0s) {
            std::this_thread::sleep_for(sleepTime);
        }
        then = std::chrono::steady_clock::now();
    }

}

} 