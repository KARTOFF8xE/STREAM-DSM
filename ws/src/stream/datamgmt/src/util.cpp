#include "util.hpp"
#include "datamgmt/common.hpp"

#include <cstring>
#include <fcntl.h>

template<typename T>
char* serialize(const T &structure) {
    char* buffer = new char[sizeof(T)];
    std::memcpy(buffer, &structure, sizeof(T));
    return buffer;
}
template char* serialize(const Client &);

template<typename T>
T deserialize(char *buffer) {
    T d;
    std::memcpy(&d, buffer, sizeof(T));
    return d;
}
template Client deserialize(char *);