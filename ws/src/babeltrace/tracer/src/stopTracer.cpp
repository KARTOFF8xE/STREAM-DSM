#include <iostream>
#include <string.h>

#include <lttng/lttng.h>

void stopTracing(std::string session) {
  std::string cmd = "lttng stop "+session;
  system(cmd.c_str());
}

void destroyTracer(std::string session) {
  std::string cmd = "lttng destroy "+session;
  system(cmd.c_str());
}

int main() {
  stopTracing("lttng_tracing");
  destroyTracer("lttng_tracing");

  return 0;
}

