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

ipc_client::ipc_client() {
    this->key_location = KEY_LOCATION;
}

void ipc_client::createKeyLocation() {
    if (fs::exists(this->key_location))
        assert(fs::remove_all(this->key_location));
    fs::create_directory(this->key_location);
}

void ipc_client::createKey() {
    this->key = ftok(this->key_location.c_str(), 0);
    this->msgQueueId = msgget(this->key, 0666 | IPC_CREAT);
    if (this->msgQueueId == -1) {
        printError;
        cleanup();
    }
}

int ipc_client::send(char msg[32]) {
    Request request;
    request.senderId = ::getpid();
    strcpy(
        request.name,
        msg
    );

    int status = msgsnd(msgQueueId, &request, sizeof(Request), IPC_NOWAIT);
    if (status == -1) {
        if (errno == EAGAIN) {
            std::clog \
            << "Failed sending request: " << std::strerror(errno) << '\n';
        }
        else {
            printError;
            return EXIT_FAILURE;
        }
    }

    return status;
}

int ipc_client::receive() {
    Request request {
        .senderId = 1,
        .name = {'\0', '\0', '\0', '\0', '\0', '\0',
        '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0'}
    };

    ssize_t nrBytes = msgrcv(this->msgQueueId, &request, sizeof(Request), 7812730813493142063, MSG_EXCEPT);
    if (nrBytes == -1) {
        printError;
        cleanup();
        return 1;
    }

    // memcpy(
    //     response,
    //     request.name,
    //     sizeof(request.name)
    // );

    // for (int i = 0; i < sizeof(request.name); i++) {
    //     response[i] = request.name[i];
    // }


    std::cout \
        << "Received request:\n"
            "\t.senderId = " << request.senderId << "\n"
            "\tresp = \"" << request.name << "\"\n";
            // "\tresponse = \"" << response << "\"\n";

    return 0;
}

void ipc_client::cleanup(int returnCode) {
    assert(fs::remove_all(this->key_location));
    std::exit(returnCode);
}