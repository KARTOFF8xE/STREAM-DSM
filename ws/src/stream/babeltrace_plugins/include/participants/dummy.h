#pragma once

#include "interface.h"

class Dummy: public IParticipant {
    public:
        /**
         * @brief Extracts the information of a trace event (trace message).
         *
         * @param event The event to extract.
         */
        void extractInfo(const bt_event *) override;

        /**
         * @brief Builds the payload used to query Graph-DB.
         *
         * @return The payload.
         */
        std::string getPayload() override;

        /**
         * @brief Sends Dummy to Graph.
         * 
         * TODO
         */
        void toGraph(std::string payload) override;
};
