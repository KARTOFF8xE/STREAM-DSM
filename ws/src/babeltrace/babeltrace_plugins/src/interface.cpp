#pragma once

#include <string>
#include <babeltrace2/babeltrace.h>

class Participant {
 public:
  virtual ~Participant() {}
  virtual void extractInfo(const bt_event *event) = 0;
  virtual std::string getPayload() = 0;
  virtual void toGraph() = 0;
};