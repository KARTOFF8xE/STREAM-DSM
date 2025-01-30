#pragma once

#include <fmt/core.h>
#include <string>
#include <iostream>

#include "../interface.h"
#include "../curl.cpp"

class FooDummy: public IParticipant {
    private:
        std::string getPayload() override {
            return "";
        }

    public:
        void extractInfo(const bt_event *) override {
            std::cout << "Found unknown Type, not handled" << std::endl;
        }

        void toGraph() override{}
};
