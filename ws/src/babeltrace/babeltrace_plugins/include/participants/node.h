#pragma once

#include "interface.h"

class Node: public IParticipant {
    private:
        std::string name;
        std::string nameSpace;
        u_int64_t   handle;
        u_int32_t   pid;

        std::string getPayload() override;
        std::string getFullName();

    public:
        void extractInfo(const bt_event *) override;
        void toGraph() override;
};
