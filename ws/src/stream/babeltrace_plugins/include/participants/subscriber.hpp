#pragma once

#include <ipc/common.hpp>

#include "interface.hpp"


class Subscriber: public IParticipant {
    private:
        std::string     name;
        u_int64_t       node_handle;
        primaryKey_t    node_primaryKey;
        primaryKey_t    primaryKey;

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
        std::string getGraphPayload() override;

        /**
         * @brief Sends a subscriber by a query to a Graph-DB.
         *
         * @param payload The query to be sent to the Graph-DB.
         */
        void toGraph(std::string payload) override;

        /**
         * @brief Builds the payload used to query Timeseries-DB.
         *
         * @return The payload.
         */
        std::string getTimeSeriesPayload() override;

        /**
         * @brief Sends a client by a query to a Timeseries-DB.
         *
         * @param payload The query to be sent to the Timeseries-DB.
         */
        void toTimeSeries(std::string payload) override;

        /**
         * @brief Sends a node switch response message.
         *
         * @param communication Reference to a Communication object used for sending the response.
         * @param enabled Enables/Disables if messages are being send.
         */
        // void response(Communication &communication);
};