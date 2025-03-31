#pragma once

#include <vector>

#include <ipc/common.hpp>

#include "interface.hpp"


class Service: public IParticipant {
    private:
        std::string                 name;
        u_int64_t                   node_handle;
        primaryKey_t                primaryKey;
        std::vector<primaryKey_t>   client_primaryKeys;
        bool                        isAction = false;

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
         * @brief Sends a service by a query to a Neo4j database.
         *
         * @param payload The query to be sent to the Neo4j database.
         */
        void toGraph(std::string payload) override;

        // TODO
        virtual std::string getTimeSeriesPayload() override;

        // TODO
        virtual void toTimeSeries(std::string payload) override;

        /**
         * @brief Sends a node switch response message.
         *
         * @param communication Reference to a Communication object used for sending the response.
         * @param enabled Enables/Disables if messages are being send.
         */
        void response(Communication &communication);
};
