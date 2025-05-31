#include "datamgmt/utils.hpp"

#include <unistd.h>
#include <limits.h>
#include <string>
#include <cstring>


std::string getHostname() {
    char hostname[HOST_NAME_MAX];
    gethostname(hostname, HOST_NAME_MAX);
    return hostname;
}

std::string getFullName(std::string name, std::string nameSpace) {
    if (nameSpace == "/") {
        return nameSpace + name;
    }

    return nameSpace + "/" + name;
}

void truncateAtSubstring(char* str, const char* substr) {
    char* pos = std::strstr(str, substr);
    if (pos != nullptr) {
        *pos = '\0';
    }
}

void truncateAfterSubstring(char* str, const char* substr) {
    char* pos = std::strstr(str, substr);
    if (pos != nullptr) {
        pos += std::strlen(substr);
        std::memmove(str, pos, std::strlen(pos) + 1);
    }
}