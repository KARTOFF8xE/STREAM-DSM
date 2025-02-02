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
 public:
  // virtual ~IParticipant() = 0;
  virtual void extractInfo(const bt_event *event) = 0;
  virtual std::string getPayload() = 0;
  virtual void toGraph() = 0;
};