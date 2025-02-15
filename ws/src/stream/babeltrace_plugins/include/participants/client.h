#pragma once

#include "interface.h"

class Client: public IParticipant {
    private:
        std::string name;
        u_int64_t node_handle;

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
         * @brief Sends Client to Graph.
         * 
         * TODO
         */
        void toGraph(std::string payload) override;
};
