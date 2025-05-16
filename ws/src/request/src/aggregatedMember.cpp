#include <iostream>

#include <ipc/ipc-client.hpp>

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
  while (true) {
    {
      std::optional<AggregatedMemberResponse> optResp = client.receiveAggregatedMemberResponse(false);
      if (optResp.has_value()) {
        AggregatedMemberResponse resp = optResp.value();
        std::cout <<
          "Received Node Reponse:" <<
          "\n\t" << resp.number << " of " << resp.total << ": " << resp.primaryKey <<
          std::endl;
      }
    }
  }
}