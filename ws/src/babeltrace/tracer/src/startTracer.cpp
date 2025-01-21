#include <iostream>
#include <string.h>

#include <lttng/lttng.h>

void createSession(std::string session, std::string path) {
  std::string cmd = "lttng create " + session + " " + " --output " + path;
  system(cmd.c_str());
}

void enableEvent(std::string eventName) {
  std::string cmd = "lttng enable-event --userspace " + eventName;
  system(cmd.c_str());
}

void addContext(std::string context) {
  std::string cmd = "lttng add-context --userspace --type=" + context;
  system(cmd.c_str());
}

void enableRotation(std::string session, int time) {
  std::string cmd = "lttng enable-rotation --session="+session+" --timer="+std::to_string(time);
  system(cmd.c_str());
}

void startTracing() {
  system("lttng start");
}

int main() {
  createSession("lttng_tracing", "/workspaces/DiplArbeitContainer/tmp/");
  enableEvent("ros2:rcl_node_init");
  addContext("vpid");
  addContext("procname");
  enableRotation("lttng_tracing", 1000000);
  startTracing();

  return 0;
}

