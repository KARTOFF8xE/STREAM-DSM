#include "datamgmt/utils.hpp"

#include <unistd.h>
#include <limits.h>
#include <string>


std::string getHostname() {
    char hostname[HOST_NAME_MAX];
    gethostname(hostname, HOST_NAME_MAX);
    return hostname;
}