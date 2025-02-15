#pragma once

#include <ipc/ipc-server.hpp>

#include "interface.h"

#include <sink.h>

class Node: public IParticipant {
    private:
        std::string name;
        std::string nameSpace;
        u_int64_t   handle;
        u_int32_t   pid;
        u_int32_t   bootcounter;

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
         * @brief Sends Node to Graph.
         * 
         * TODO
         */
        void toGraph(std::string payload) override;

        /**
         * // TODO
         */
        void response(Communication &communication, bool enabled);
};
