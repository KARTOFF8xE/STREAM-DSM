#include "util.hpp"
#include "datamgmt/common.hpp"

#include <cstring>
#include <fcntl.h>

template char* serialize<Data>(const Data &);
template Data deserialize(const char*);

template<typename T>
char* serialize(const T &structure) {
    char* buffer = new char[sizeof(T)];
    std::memcpy(buffer, &d, sizeof(T));
    return buffer;
}

template<typename T>
T deserialize(const char *buffer) {
    T d;
    std::memcpy(&d, buffer, sizeof(T));
    return d;
}