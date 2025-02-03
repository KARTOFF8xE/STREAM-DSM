#pragma once

#include <iostream>
#include <filesystem>
namespace fs = std::filesystem;

#include <cassert>
#include <cstdlib>
#include <cerrno>
#include <cstring>

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>

#include "common.h"

class ipc_client {
    private:
        fs::path key_location;
        key_t key;
        int msgQueueId;

    public:
        ipc_client();

        void createKeyLocation();

        void createKey();

        int send(char msg[32]);

        int receive();

        void cleanup(int returnCode = EXIT_FAILURE);
};