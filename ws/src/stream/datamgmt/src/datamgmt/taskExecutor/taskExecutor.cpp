#include <iostream>
#include <vector>
#include <variant>
#include <bits/stdc++.h>
#include <unordered_set>
#include <nlohmann/json.hpp>
#include <chrono>
using namespace std::chrono_literals;

#include <ipc/ipc-client.hpp>
#include <ipc/util.hpp>
#include <curl/myCurl.hpp>
#include <neo4j/tree/tree.hpp>
#include <influxdb/influxdb.hpp>

#include "datamgmt/taskExecutor/taskExecutor.hpp"
#include "pipe/pipe.hpp"
#include "ipc/common.hpp"


using json = nlohmann::json;


namespace taskExecutor {

// bool receiveIPCClientRequest(const IpcServer &ipcServer, SingleAttributeInformationRequest &receivedRequest) {
//     std::optional<SingleAttributesRequest> response =
//         ipcServer.receiveSingleAttributesRequest(receivedRequest.requestID, receivedRequest.pid, false);

//     if (response.has_value()) {
//         receivedRequest.payload = response.value();

//         return true;
//     }

//     return false;
// }

double getValueStandardQueryInfluxDB(Task &task) {
    std::string request;
    if (std::holds_alternative<SingleAttributesRequest>(task.task)) {
        request =
            influxDB::createPayloadGetSingleValue(
                "STREAM",
                std::get<SingleAttributesRequest>(task.task).attribute,
                task.primaryKeys);
    }
    if (std::holds_alternative<AggregatedAttributesRequest>(task.task)) {
        request =
            influxDB::createPayloadGetSingleValue(
                "STREAM",
                std::get<AggregatedAttributesRequest>(task.task).attribute,
                task.primaryKeys);
    }

    std::string response = curl::push(request, curl::INFLUXDB_READ);

    return influxDB::extractValueFromCSV(response);
}

std::vector<std::string> split(std::string str, char delimiter) {
    std::stringstream ss(str);
    std::vector<std::string> res;
    std::string token;
    while (std::getline(ss, token, delimiter)) {
        res.push_back(token);
    }

    return res;
}

std::vector<std::string> getCustomResponseQueryInfluxDB(Task &task) {
    std::string response =
        curl::push(
            std::get<CustomAttributesTask>(task.task).query,
            curl::INFLUXDB_READ);
    
    return split(response, '\n');
}

std::vector<std::string> getCustomResponseQueryNeo4J(Task &task) {
    std::string query = R"(
          {
              "statements":
                  [
                      { "statement": ")";
    query += std::get<CustomMemberTask>(task.task).query + R"( "
                      }
                  ]
          })";
    std::string response = curl::push(query, curl::NEO4J);

    std::vector<std::string> responseSplitted;

    for (std::size_t i = 0; i < response.length(); i += MAX_STRING_SIZE-1) {
        responseSplitted.push_back(response.substr(i, MAX_STRING_SIZE-1));
    }
    
    return responseSplitted;
}

void taskExecutor(std::map<Module_t, pipe_ns::Pipe> pipes, std::atomic<bool> &running, Tasks &tasks) {
    std::cout << "started taskExecutor" << std::endl;

    auto then = std::chrono::steady_clock::now();
    while (true) {
        {
            std::lock_guard<std::mutex> lock(tasks.mutex);
            for (Task &task : tasks.vec) {
                switch (task.type)
                {
                case SINGLEATTRIBUTE:
                {
                    task.channel->send(
                        sharedMem::Response {
                            .header {
                                .type           = sharedMem::NUMERICAL,
                                .payloadSize    = sizeof(sharedMem::NumericalResponse)
                            },
                            .payload {
                                .numerical {
                                    .number = 1,
                                    .total  = 1,
                                    .value  = getValueStandardQueryInfluxDB(task)
                                }
                            }
                        }
                    );

                    if (!std::get<SingleAttributesRequest>(task.task).continuous) {
                        tasks.vec.erase(find(tasks.vec.begin(), tasks.vec.end(), task));
                    }
                    break;
                }
                case AGGREGATEDATTRIBUTE:
                {
                    task.channel->send(
                        sharedMem::Response {
                            .header {
                                .type           = sharedMem::NUMERICAL,
                                .payloadSize    = sizeof(sharedMem::NumericalResponse)
                            },
                            .payload {
                                .numerical {
                                    .number = 1,
                                    .total  = 1,
                                    .value  = getValueStandardQueryInfluxDB(task)
                                }
                            }
                        }
                    );

                    if (!std::get<AggregatedAttributesRequest>(task.task).continuous) {
                        tasks.vec.erase(find(tasks.vec.begin(), tasks.vec.end(), task));
                    }
                    break;
                }
                case CUSTOMATTRIBUTE:
                {
                    std::vector<std::string> response = getCustomResponseQueryInfluxDB(task);

                    size_t counter = 0;
                    for (auto line: response) {
                        sharedMem::Response resp {
                            .header {
                                .type           = sharedMem::TEXTUAL,
                                .payloadSize    = sizeof(sharedMem::TextualResponse),
                            },
                            .payload {
                                .textual {
                                    .number = ++counter,
                                    .total  = response.size()
                                }
                            }
                        };
                        util::parseString(resp.payload.textual.line, line);

                        task.channel->send(resp);
                    }

                    if (!std::get<CustomAttributesTask>(task.task).continuous) {
                        tasks.vec.erase(find(tasks.vec.begin(), tasks.vec.end(), task));
                    }
                    break;
                }
                case AGGREGATEDMEMBER:
                {
                    u_int16_t counter = 0;
                    for (auto primKey: task.primaryKeys) {
                        task.channel->send (
                                sharedMem::Response {
                                .header {
                                    .type           = sharedMem::NUMERICAL,
                                    .payloadSize    = sizeof(sharedMem::NumericalResponse)
                                },
                                .payload {
                                    .numerical  {
                                        .number = ++counter,
                                        .total  = task.primaryKeys.size(),
                                        .value  = double(primKey)
                                    }
                                }
                            }
                        );
                    }

                    if (!std::get<AggregatedMemberRequest>(task.task).continuous) {
                        tasks.vec.erase(find(tasks.vec.begin(), tasks.vec.end(), task));
                    }

                    break;
                }
                case CUSTOMMEMBER:
                {
                    std::vector<std::string> response = getCustomResponseQueryNeo4J(task);

                    size_t counter = 0;
                    for (auto line: response) {
                        sharedMem::Response resp {
                            .header {
                                .type           = sharedMem::TEXTUAL,
                                .payloadSize    = sizeof(sharedMem::TextualResponse)
                            },
                            .payload {
                                .textual {
                                    .number = ++counter,
                                    .total  = response.size()
                                }
                            }
                        };
                        util::parseString(resp.payload.textual.line, line);

                        task.channel->send(resp);
                    }

                    if (!std::get<CustomMemberTask>(task.task).continuous) {
                        tasks.vec.erase(find(tasks.vec.begin(), tasks.vec.end(), task));
                    }
                    break;
                }
                default:
                    break;
                }
            }
        }

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now-then);
        auto sleepTime = 1s - elapsed;
        if (sleepTime > 0s) {
            std::this_thread::sleep_for(sleepTime);
        }
        then = std::chrono::steady_clock::now();
    }
}

}