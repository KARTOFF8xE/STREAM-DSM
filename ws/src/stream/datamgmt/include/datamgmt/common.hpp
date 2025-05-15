#pragma once

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
    StandardSingleAttributesRequest payload;
};