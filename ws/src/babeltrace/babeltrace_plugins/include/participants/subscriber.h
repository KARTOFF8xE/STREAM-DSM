#pragma once

#include "interface.h"

class FooSubscriber: public IParticipant {
    private:
        std::string name;
        u_int64_t   pubsub_handle;
        u_int64_t   node_handle;

        std::string getPayload() override;

    public:
        void extractInfo(const bt_event *event) override;
        void toGraph() override;
};