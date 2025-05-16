#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <memory>
#include <cstring>
#include "pipe/pipe.hpp"

#include "pipe/pipe.hpp"
#include "datamgmt/datamgmt.hpp"
#include "datamgmt/common.hpp"


int getPipe(int p[2], bool blocking) {
    if (int ret = pipe(p); ret != 0) {
        std::cerr << "\e[31mError: Failed to create pipe. Code: " << ret << "\e[0m" << std::endl;
        return ret;
    }

    if (!blocking) {
        int ret = fcntl(p[0], F_SETFL, fcntl(p[0], F_GETFL) | O_NONBLOCK);
        if (ret != 0) {
            std::cerr << "\e[31mError: Failed to set pipe non-blocking. Code: " << ret << "\e[0m" << std::endl;
            return ret;
        }
    }

    return 0;
}

template<typename T>
ssize_t writeT(int __fd, const T& payload, MsgType type) {
    Header header{type, static_cast<uint32_t>(sizeof(T))};
    size_t totalSize = sizeof(Header) + sizeof(T);
    std::unique_ptr<char[]> msg = std::make_unique<char[]>(totalSize);

    std::memcpy(msg.get(), &header, sizeof(Header));
    std::memcpy(msg.get() + sizeof(Header), &payload, sizeof(T));

    int ret = write(__fd, msg.get(), totalSize);
    if (ret == -1) {
        std::cerr << "\e[31mError: Failed to write pipe.\e[0m" << std::endl;
    }

    return ret;
}

template<typename T>
ssize_t readT(int __fd, T& payload, MsgType* type) {
    Header header;
    ssize_t hret = read(__fd, &header, sizeof(Header));
    if (hret <= 0) return -1;

    if (type) *type = header.type;

    // if (header.size != sizeof(T)) {
    //     std::cerr << "\e[31mError: Payload size mismatch.\e[0m" << std::endl;
    //     return -1;
    // }

    ssize_t pret = read(__fd, &payload, sizeof(T));
    if (pret <= 0) return -1;

    return pret;
}

// Explizite Instanziierungen:
template ssize_t writeT<Client>(int, const Client&, MsgType);
template ssize_t writeT<NodeResponse>(int, const NodeResponse&, MsgType);
template ssize_t writeT<SingleStandardInformationRequest>(int, const SingleStandardInformationRequest&, MsgType);
template ssize_t writeT<AggregatedStandardInformationRequest>(int, const AggregatedStandardInformationRequest&, MsgType);
template ssize_t writeT<CustomInformationRequest>(int, const CustomInformationRequest&, MsgType);
template ssize_t writeT<union_Tasks>(int, const union_Tasks&, MsgType);

template ssize_t readT<Client>(int, Client&, MsgType*);
template ssize_t readT<NodeResponse>(int, NodeResponse&, MsgType*);
template ssize_t readT<SingleStandardInformationRequest>(int, SingleStandardInformationRequest&, MsgType*);
template ssize_t readT<AggregatedStandardInformationRequest>(int, AggregatedStandardInformationRequest&, MsgType*);
template ssize_t readT<CustomInformationRequest>(int, CustomInformationRequest&, MsgType*);
template ssize_t readT<union_Tasks>(int, union_Tasks&, MsgType*);
