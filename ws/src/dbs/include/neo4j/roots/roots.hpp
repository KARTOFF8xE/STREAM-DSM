#pragma once

#include <string>
#include <fmt/core.h>


namespace createRoot {

    /**
     * @brief Builds the payload used to query neo4j.
     * 
     * @param fullName The full name of the Node (namespace+name).
     * 
     * @returns The Payload for the Query.
     */
    std::string getPayloadCreateNameSpaceAndLinkPassiveHelpers(std::string fullName);
    
    /**
     * @brief Builds the payload used to query neo4j.
     * 
     * @param pids The pids up to the root in a list, i.e. '["1", "2", "3"]'.
     * 
     * @returns The Payload for the Query.
     */
    std::string getPayloadCreateProcessAndLinkPassiveHelpers(std::string pids);

    /**
     * @brief Builds the payload used to query neo4j.
     * 
     * @param fullName The full name of the Node (namespace+name).
     * @param pids The pids up to the root in a list, i.e. '["1", "2", "3"]'.
     * 
     * @returns The Payload for the Query.
     */
    std::string getPayloadCreateProcessAndUpdatePassiveHelpers(std::string fullName, std::string pids);

    /**
     * @brief Builds the payload used to query neo4j.
     * 
     * @param hostName The name of the PC.
     * @param macAdress The MAC-Adress of the PC.
     * 
     * @returns The Payload for the Query.
     */
    std::string createRoot(std::string hostName, std::string macAdress);
}