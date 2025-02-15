#pragma once

#include "interface.hpp"

class Subscriber: public IParticipant {
    private:
        std::string name;
        u_int64_t   pubsub_handle;
        u_int64_t   node_handle;

    public:
        /**
         * @brief Extracts the information of a trace event (trace message).
         *
         * @param event The event to extract.
         */
        void extractInfo(const bt_event *event) override;

        /**
         * @brief Builds the payload used to query Graph-DB.
         *
         * @return The payload.
         */
        std::string getPayload() override;

        /**
         * @brief Sends a subscriber by a query to a Neo4j database.
         *
         * @param payload The query to be sent to the Neo4j database.
         */
        void toGraph(std::string payload) override;
};