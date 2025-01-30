#pragma once

#include "interface.h"

class FooDummy: public IParticipant {
    private:
        std::string getPayload() override;

    public:
        void extractInfo(const bt_event *) override;
        void toGraph() override;
};
