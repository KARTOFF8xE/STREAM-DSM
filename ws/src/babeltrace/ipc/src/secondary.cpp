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
#include <unistd.h>

#include "common.h"
#include "ipc/ipc-client.h"


int main()
{
    ipc_client client;
    client.createKey();
    // Request request;
    // request.senderId = ::getpid();
    char msg[32];
    std::cout << ">>> " << std::flush;
    ssize_t nrBytes = ::read(STDIN_FILENO, msg, sizeof(Request::name));
    if (nrBytes == -1) {
        printError;
    }

    // replace \n with \0
    // request.name[nrBytes - 1] = '\0';
    msg[nrBytes - 1] = '\0';
    client.send(msg);

  return EXIT_SUCCESS;
}
