#pragma once

#include <vector>

#include <ipc/common.hpp>
#include <ipc/ipc-server.hpp>

#define STRINGSIZE 16


struct Client {
    pid_t           pid;
    requestId_t     requestId;
    primaryKey_t    primaryKey;
    bool            updates;

    bool operator==(const Client& other) const {
        return this->pid == other.pid;
    }
};
// #define MSGSIZE sizeof(Client)


struct SingleStandardInformationRequest {
    requestId_t                         requestID;
    pid_t                               pid;
    StandardSingleAttributesRequest     payload;
};


struct AggregatedStandardInformationRequest {
    requestId_t                         requestID;
    pid_t                               pid;
    StandardAggregatedAttributesRequest payload;
};

struct CustomInformationRequest {
    requestId_t             requestID;
    pid_t                   pid;
    CustomAttributesRequest payload;
};

struct StandardInformationMemberRequest {
    requestId_t             requestID;
    pid_t                   pid;
    AggregatedMemberRequest payload;
};

struct CustomInformationMemberRequest {
    requestId_t             requestID;
    pid_t                   pid;
    CustomMemberRequest payload;
};

union union_Requests {
    SingleStandardInformationRequest        standardSingle;
    AggregatedStandardInformationRequest    standardAggregated;
    CustomInformationRequest                custom;
    StandardInformationMemberRequest        standardMember;
    CustomInformationMemberRequest          customMember;

    union_Requests() {}
    ~union_Requests() {}
};