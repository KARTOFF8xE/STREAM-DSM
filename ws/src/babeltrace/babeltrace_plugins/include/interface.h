#pragma once

#include <string>
#include <babeltrace2/babeltrace.h>

struct Request {
    std::string url;
    std::string username;
    std::string password;
    std::string query_request;
    std::string query_response;
};

class IParticipant {

private:
/**
 * @brief Builds the payload used to query Graph-DB.
 *
 * @return The payload.
 */
  virtual std::string getPayload() = 0;

public:
  
/**
 * @brief Extracts the information of a given trace event (trace message).
 *
 * @param event The event to extract.
 */
  virtual void extractInfo(const bt_event *event) = 0;

/**
 * @brief Sends component to Graph.
 */
  virtual void toGraph() = 0;
};