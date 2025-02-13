#include <iostream>
#include <unistd.h>
#include <fcntl.h>

#include "pipe/pipe.hpp"
#include "datamgmt/common.hpp"
#include "util.hpp"

int getPipe(int p[2], bool blocking = false) {
    if (int ret = pipe(p) != 0) {
        std::cerr << "\e[31mError: Failed to create pipe. Code: " << ret << "\e[30m" << std::endl;
        return ret;
    }

    if (!blocking) {
        int ret = fcntl( p[0], F_SETFL, fcntl(p[0], F_GETFL) | O_NONBLOCK);
        if (ret != 0) {
            std::cerr << "\e[31mError: Failed to set pipe not blocking. Code: " << ret << "\e[30m" << std::endl;
            return ret;
        }
    }

    return 0;
}

template<typename T>
ssize_t writeT(int __fd, const T payload, size_t __n) {
    char *msg = serialize<Data>(payload);
    
    int ret = write(__fd, msg, __n);
    if (ret == -1) {
        std::cerr << "\e[31mError: Failed to write pipe.\e[30m" << std::endl;
    }
    
    return ret;
}
template ssize_t writeT<Data>(int, const Data, size_t);

template<typename T>
ssize_t readT(int __fd, T &payload, size_t __n) {
    char *msg = new char[__n];
    int ret = read(__fd, msg, __n);
    if (ret == -1) {
        std::cerr << "\e[31mError: Failed to read pipe.\e[30m" << std::endl;
        return ret;
    }

    payload = deserialize<Data>(msg);

    return 0;
}
template ssize_t readT<Data>(int, Data &, size_t);