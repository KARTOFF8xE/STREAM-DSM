#pragma once

#include <ipc/common.hpp>

#define STRINGSIZE 16

struct Client {
    pid_t pid;
    requestId_t requestId;
    bool subState;
};

struct Data {
    Client client;
    bool subscribed;
};
#define MSGSIZE sizeof(Data)