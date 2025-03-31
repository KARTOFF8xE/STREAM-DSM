#include <fmt/core.h>
#include <string>
#include <iostream>

#include "interface.hpp"
#include "participants/dummy.hpp"


void Dummy::extractInfo(const bt_event *) {
    std::cout << "Found unknown Type, not handled" << std::endl;
}

std::string Dummy::getGraphPayload() {
    return "";
}

void Dummy::toGraph(std::string payload) {}

std::string Dummy::getTimeSeriesPayload() {return "";}

void Dummy::toTimeSeries(std::string payload) {}