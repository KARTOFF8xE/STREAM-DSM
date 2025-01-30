#include <fmt/core.h>
#include <string>
#include <iostream>

#include "interface.h"
#include "participants/dummy.h"
#include "curl.h"

std::string FooDummy::getPayload() {
    return "";
}

void FooDummy::extractInfo(const bt_event *) {
    std::cout << "Found unknown Type, not handled" << std::endl;
}

void FooDummy::toGraph() {}
