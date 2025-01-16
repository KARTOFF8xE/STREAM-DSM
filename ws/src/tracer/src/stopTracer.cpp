#include <iostream>
#include <string.h>

#include <lttng/lttng.h>

void stopTracing() {
  system("lttng stop");
}

void destroyTracer() {
  system("lttng destroy");
}


int main() {
  stopTracing();
  destroyTracer();

  return 0;
}

