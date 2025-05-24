#pragma once

#include <cstdint>
#include <unistd.h>
#include <sys/types.h>
#include "datamgmt/common.hpp"


namespace pipe_ns {

enum MsgType {
    NONE,
    NODERESPONSE,
    TOPICRESPONSE
};

struct Header {
    MsgType type;
    uint32_t size;
};

struct Pipe {
    int read;
    int write;
};

union UnionResponse {
    TopicResponse   topicResp;
    NodeResponse    nodeResp;

    UnionResponse() {}
};

int getPipe(int p[2], bool blocking = false);

template<typename T>
ssize_t writeT(int __fd, const T& payload, MsgType type = MsgType::NONE);

template<typename T>
ssize_t readT(int __fd, T& payload, MsgType* type = nullptr);

}