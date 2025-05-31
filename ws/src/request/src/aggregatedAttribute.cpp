#include <iostream>

#include "common.hpp"

#include <ipc/ipc-client.hpp>
#include <ipc/sharedMem.hpp>

#include <chrono>
#include <thread>

int main() {
  std::cout << "Make a Aggregated Attribute Request..." << std::endl;

  IpcClient client(4);

  // std::cout << ">> ";
  // primaryKey_t pKey;
  // std::cin >> pKey;
  requestId_t requestId;
  const AggregatedAttributesRequest request{
    .rootedTree1 {
      .primaryKey = 4,
      .tree       = PROCESSDRIVEN,
    },
    .rootedTree2 {
      .primaryKey = 11,
      .tree       = PROCESSDRIVEN
    },
    .attribute = CPU_UTILIZATION,
    .binOperation = DIFFERENCE,
    .continuous = true,
  };

  std::cout << "send request..." << std::flush;
  client.sendAggregatedAttributesRequest(request, requestId, false);
  std::cout << "done" << std::endl;
  std::optional<AggregatedAttributesResponse> response = client.receiveAggregatedAttributesResponse();
  if (!response.has_value()) { std::cerr << "Response got no Value" << std::flush; return 1; }
  AggregatedAttributesResponse resp = response.value();

  sharedMem::SHMChannel<sharedMem::Response> channel(resp.memAddress, true);

  while (true) {
    {
      sharedMem::Response sharedMemResponse {};
      if (!channel.receive(sharedMemResponse, false)) continue;

      printResponse<sharedMem::NumericalResponse>(sharedMemResponse);
    }
  }
}