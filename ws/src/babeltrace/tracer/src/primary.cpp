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

#include "ipc/ipc-client.h"

// void cleanup(int returnCode = EXIT_FAILURE) {
//   assert(fs::remove_all(KEY_LOCATION));
//   std::exit(returnCode);
// }

int main() {
    ipc_client client;
    client.createKeyLocation();
    client.createKey();
    while (true) {
        client.receive();
    }

    client.cleanup(EXIT_SUCCESS);
}
