#pragma once

#include "interface.h"

class Subscriber: public IParticipant {
    private:
        std::string name;
        u_int64_t   pubsub_handle;
        u_int64_t   node_handle;

        /**
         * @brief Builds the payload used to query Graph-DB.
         *
         * @return The payload.
         */
        std::string getPayload() override;

    public:
        /**
         * @brief Extracts the information of a trace event (trace message).
         *
         * @param event The event to extract.
         */
        void extractInfo(const bt_event *event) override;

        /**
         * @brief Sends Subscriber to Graph.
         */
        void toGraph() override;
};