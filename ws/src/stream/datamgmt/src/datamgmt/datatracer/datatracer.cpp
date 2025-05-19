#include "datamgmt/datatracer/datatracer.hpp"

#include <vector>
#include <iostream>
#include <map>
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

    std::vector<sharedMem::SHMChannel> channels;

    auto then = std::chrono::steady_clock::now();
    while (true) {
        requestId_t requestID;
        pid_t pid;
        std::optional<SHMAddressRequest> request = server.receiveSHMAddressRequest(requestID, pid, false);
        if (request.has_value()) {
            SHMAddressRequest payload = request.value();

            char address[MAX_STRING_SIZE];
            sprintf(address, "/%d%d", pid, requestID);
            SHMAddressResponse response;

            util::parseString(response.memAdress, address);

            channels.push_back(sharedMem::SHMChannel(address, true));

            continue;
        }

        size_t counter = 0;
        std::vector<sharedMem::Value> values;
        std::vector<influxDB::ValuePairs> valuePairs;
        for (auto channel : channels) {
            sharedMem::Value msg;
            channel.receive(msg);
            values.push_back(msg);
            valuePairs.push_back(
                influxDB::ValuePairs {
                    .primaryKey = msg.primaryKey,
                    .timestamp  = msg.timestamp,
                    .value      = msg.value,
                }
            );

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

            if (counter++ >= 10 || 
                std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-then) >= 1s
            ) {
                counter = 0;

                curl::push(influxDB::createPayloadMultipleValSameTime(valuePairs), curl::INFLUXDB_WRITE);
                valuePairs.clear();
            }
        }

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