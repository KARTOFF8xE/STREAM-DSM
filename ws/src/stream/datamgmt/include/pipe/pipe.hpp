#pragma once

#include <cstdint>
#include <unistd.h>
#include <sys/types.h>

enum MsgType {
    NONE,
    STANDARDSINGLE,
    STANDARDAGGREGATED,
    CUSTOM,
};

struct Header {
    MsgType type;
    uint32_t size;
};

struct Pipe {
    int read;
    int write;
};

int getPipe(int p[2], bool blocking = false);

template<typename T>
ssize_t writeT(int __fd, const T& payload, MsgType type = MsgType::NONE);

template<typename T>
ssize_t readT(int __fd, T& payload, MsgType* type = nullptr);
