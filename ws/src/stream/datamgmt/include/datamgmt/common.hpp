#pragma once

#include <ipc/common.hpp>

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
#define MSGSIZE sizeof(Client)