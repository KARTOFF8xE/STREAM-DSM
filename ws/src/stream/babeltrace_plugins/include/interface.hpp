#pragma once

#include <string>
#include <babeltrace2/babeltrace.h>

#include "common.hpp"

struct Request {
    std::string url;
    std::string username;
    std::string password;
    std::string query_request;
    std::string query_response;
};

class IParticipant {
public:
  
  /**
   * @brief Extracts the information of a given trace event (trace message).
   *
   * @param event The event to extract.
   */
  virtual void extractInfo(const bt_event *event) = 0;

  /**
   * @brief Builds the payload used to query Graph-DB.
   *
   * @return The payload.
   */
  virtual std::string getPayload() = 0;
  
  /**
   * @brief Sends a request by a query to a Neo4j database.
   *
   * @param payload The query to be sent to the Neo4j database.
   */
  virtual void toGraph(std::string payload) = 0;

  /**
   * @brief Sends a response message.
   *
   * @param communication Reference to a Communication object used for sending the response.
   * @param enabled Enables/Disables if messages are being send.
   */
  virtual void response(Communication &communication) {}
};