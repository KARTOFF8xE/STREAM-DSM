#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <memory>
#include <cstring>
#include "pipe/pipe.hpp"

#include "pipe/pipe.hpp"
#include "datamgmt/datamgmt.hpp"


namespace pipe_ns {

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

    ssize_t pret = read(__fd, &payload, sizeof(T));
    if (pret <= 0) return -1;

    return pret;
}

// Explizite Instanziierungen:
template ssize_t writeT<RequestingClient>(int, const RequestingClient&, MsgType);
template ssize_t writeT<NodeResponse>(int, const NodeResponse&, MsgType);
template ssize_t writeT<UnionResponse>(int, const UnionResponse&, MsgType);
template ssize_t writeT<SingleAttributeInformationRequest>(int, const SingleAttributeInformationRequest&, MsgType);
template ssize_t writeT<AggregatedAttributeInformationRequest>(int, const AggregatedAttributeInformationRequest&, MsgType);
template ssize_t writeT<CustomAttributeInformationRequest>(int, const CustomAttributeInformationRequest&, MsgType);
template ssize_t writeT<AggregatedMemberInformationRequest>(int, const AggregatedMemberInformationRequest&, MsgType);
template ssize_t writeT<CustomMemberInformationRequest>(int, const CustomMemberInformationRequest&, MsgType);
template ssize_t writeT<union_Requests>(int, const union_Requests&, MsgType);

template ssize_t readT<RequestingClient>(int, RequestingClient&, MsgType*);
template ssize_t readT<NodeResponse>(int, NodeResponse&, MsgType*);
template ssize_t readT<UnionResponse>(int, UnionResponse&, MsgType*);
template ssize_t readT<SingleAttributeInformationRequest>(int, SingleAttributeInformationRequest&, MsgType*);
template ssize_t readT<AggregatedAttributeInformationRequest>(int, AggregatedAttributeInformationRequest&, MsgType*);
template ssize_t readT<CustomAttributeInformationRequest>(int, CustomAttributeInformationRequest&, MsgType*);
template ssize_t readT<AggregatedMemberInformationRequest>(int, AggregatedMemberInformationRequest&, MsgType*);
template ssize_t readT<CustomMemberInformationRequest>(int, CustomMemberInformationRequest&, MsgType*);
template ssize_t readT<union_Requests>(int, union_Requests&, MsgType*);

}