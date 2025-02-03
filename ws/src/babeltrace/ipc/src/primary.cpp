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
#include "ipc/ipc-client.h"

int main() {
    ipc_client client;
    client.createKeyLocation();
    client.createKey();

    char msg[32];
    while (true) {
        client.receive();
    }

    client.cleanup(EXIT_SUCCESS);
}
