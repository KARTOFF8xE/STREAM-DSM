#pragma once

#include <vector>

#include <ipc/common.hpp>
#include <ipc/ipc-server.hpp>

#define STRINGSIZE 16


struct RequestingClient {
    pid_t           pid;
    requestId_t     requestId;
    primaryKey_t    primaryKey;
    bool            updates;

    bool operator==(const RequestingClient& other) const {
        return this->pid == other.pid;
    }
};
// #define MSGSIZE sizeof(Client)


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