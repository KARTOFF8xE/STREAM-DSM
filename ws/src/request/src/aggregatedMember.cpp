#include <iostream>

#include <ipc/ipc-client.hpp>
#include <ipc/sharedMem.hpp>

#include <chrono>
#include <thread>

int main() {
  std::cout << "Make a Standard Aggregated Attribute Request..." << std::endl;

  IpcClient client(4);

  // std::cout << ">> ";
  // primaryKey_t pKey;
  // std::cin >> pKey;
  requestId_t requestId;
  const AggregatedMemberRequest request{
    .primaryKey_RootTree1 = 4,
    .primaryKey_RootTree2 = 11,
    .tree1 = PROCESSDRIVEN,
    .tree2 = PROCESSDRIVEN,
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
      if (!channel.receive(sharedMemResponse)) continue;

      sharedMem::printResponse<sharedMem::NumericalResponse>(sharedMemResponse);
    }
  }
}