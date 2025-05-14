#pragma once

#include "interface.hpp"

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
        std::string getGraphPayload() override;

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
         * @brief Sends a dummy by a query to a Neo4j database.
         *
         * @param payload The query to be sent to the Neo4j database.
         */
        void toGraph(std::string payload) override;
};
