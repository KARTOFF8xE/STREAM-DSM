#pragma once

#include <vector>
#include <mutex>

#include <ipc/common.hpp>
#include <ipc/ipc-server.hpp>

#define STRINGSIZE 16
// #define TASK

extern bool gsRunning;

struct RequestingClient {
    pid_t           pid;
    requestId_t     requestId;
    std::string     primaryKey;
    bool            updates;

    bool operator==(const RequestingClient& other) const {
        return this->pid == other.pid;
    }
};


struct SingleAttributeInformationRequest {
    requestId_t             requestID;
    pid_t                   pid;
    SingleAttributesRequest payload;
};


struct AggregatedAttributeInformationRequest {
    requestId_t                 requestID;
    pid_t                       pid;
    AggregatedAttributesRequest payload;
};

struct CustomAttributeInformationRequest {
    requestId_t             requestID;
    pid_t                   pid;
    CustomAttributesRequest payload;
};

struct AggregatedMemberInformationRequest {
    requestId_t             requestID;
    pid_t                   pid;
    AggregatedMemberRequest payload;
};

struct CustomMemberInformationRequest {
    requestId_t         requestID;
    pid_t               pid;
    CustomMemberRequest payload;
};

union union_Requests {
    SingleAttributeInformationRequest       singleAttribute;
    AggregatedAttributeInformationRequest   aggregatedAttribute;
    CustomAttributeInformationRequest       customAttribute;
    AggregatedMemberInformationRequest      aggregatedMember;
    CustomMemberInformationRequest          customMember;

    union_Requests() {}
    ~union_Requests() {}
};