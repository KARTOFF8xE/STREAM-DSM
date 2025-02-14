#include <fmt/core.h>
#include <string>
#include <iostream>

#include "interface.h"
#include "participants/dummy.h"
#include "curl.h"

std::string Dummy::getPayload() {
    return "";
}

void Dummy::extractInfo(const bt_event *) {
    std::cout << "Found unknown Type, not handled" << std::endl;
}

void Dummy::toGraph() {}
