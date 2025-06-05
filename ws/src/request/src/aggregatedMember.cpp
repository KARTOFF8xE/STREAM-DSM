#include <iostream>

#include "common.hpp"

#include <ipc/ipc-client.hpp>
#include <ipc/sharedMem.hpp>

#include <chrono>
#include <thread>

int main() {
  std::cout << "Make a Standard Aggregated Attribute Request..." << std::endl;

  IpcClient client(1);

  // std::cout << ">> ";
  // primaryKey_t pKey;
  // std::cin >> pKey;
  requestId_t requestId;
  const AggregatedMemberRequest request{
    .rootedTree1 {
      .primaryKey = "d3ce1aa3-d48f-4b20-9c59-e3f4f10e0f05",
      .tree       = PROCESSDRIVEN,
    },
    .rootedTree2 {
      .primaryKey = "8524d310-3dff-4248-abef-5f12ab5d6b24",
      .tree       = PROCESSDRIVEN
    },
    .binOperation = DIFFERENCE,
    .continuous = false,
  };

  std::cout << "send request..." << std::flush;
  client.sendAggregatedMemberRequest(request, requestId, false);
  std::cout << "done" << std::endl;

  std::optional<AggregatedMemberResponse> response = client.receiveAggregatedMemberResponse();
  if (!response.has_value()) { std::cerr << "Response got no Value" << std::flush; return 1; }
  AggregatedMemberResponse resp = response.value();

  sharedMem::SHMChannel<sharedMem::Response> channel(resp.memAddress, true);

  while (true) {
    {
      sharedMem::Response sharedMemResponse {};
      if (!channel.receive(sharedMemResponse, false)) continue;

      printResponse<sharedMem::TextualResponse>(sharedMemResponse);
    }
  }
}