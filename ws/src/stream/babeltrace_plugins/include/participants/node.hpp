#pragma once

#include <ipc/ipc-server.hpp>

#include "interface.hpp"


class Node: public IParticipant {
    private:
        primaryKey_t    primaryKey;
        u_int64_t       handle;
        std::string     name;
        std::string     nameSpace;
        pid_t           pid;
        u_int32_t       bootcounter;

        /**
         * @brief Concats the namespace and the name and inserts a "/" if needed.
         *
         * @return The full name.
         */
        std::string getFullName();

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
         * @brief Sends a node by a query to a Neo4j database.
         *
         * @param payload The query to be sent to the Neo4j database.
         */
        void toGraph(std::string payload) override;

        /**
         * @brief Sends a node switch response message.
         *
         * @param communication Reference to a Communication object used for sending the response.
         * @param enabled Enables/Disables if messages are being send.
         */
        void response(Communication &communication, bool enabled);
};
