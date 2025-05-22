#pragma once

#include <string>
#include <variant>
#include <vector>
#include <mutex>
#include <ipc/common.hpp>
#include <ipc/ipc-client.hpp>
#include <ipc/sharedMem.hpp>

#include "pipe/pipe.hpp"


struct CustomAttributesTask {
    std::string query;
    bool        continuous;
};

struct CustomMemberTask {
    std::string query;
    bool        continuous;
};

enum TaskType {
    SINGLEATTRIBUTE,
    AGGREGATEDATTRIBUTE,
    CUSTOMATTRIBUTE,
    AGGREGATEDMEMBER,
    CUSTOMMEMBER,
};

using TaskVariant = std::variant<
    SingleAttributesRequest,
    AggregatedAttributesRequest,
    CustomAttributesTask,
    AggregatedMemberRequest,
    CustomMemberTask
>;

struct Task {
    pid_t                                                       pid;
    requestId_t                                                 requestId;
    TaskType                                                    type;
    std::vector<primaryKey_t>                                   primaryKeys;

    TaskVariant                                                 task;
    std::unique_ptr<sharedMem::SHMChannel<sharedMem::Response>> channel;

    bool operator==(const Task& other) const {
        return this->pid == other.pid;
    }
};

struct Tasks {
    std::vector<Task>   vec;
    std::mutex          mutex;
};