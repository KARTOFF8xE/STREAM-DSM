#include <iostream>

#include <ipc/ipc-client.hpp>

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
    .primaryKey_RootTree1 = 4,
    .primaryKey_RootTree2 = 11,
    .tree1 = PROCESSDRIVEN,
    .tree2 = PROCESSDRIVEN,
    .attribute = CPU_UTILIZATION,
    .binOperation = DIFFERENCE,
    .continuous = true,
  };

  std::cout << "send request..." << std::flush;
  client.sendAggregatedAttributesRequest(request, requestId, false);
  std::cout << "done" << std::endl;
  while (true) {
    {
      std::optional<AggregatedAttributesResponse> optResp = client.receiveAggregatedAttributesResponse(false);
      if (optResp.has_value()) {
        AggregatedAttributesResponse resp = optResp.value();
        std::cout <<
          "Received Node Reponse:" <<
          "\n\tvalue: " << resp.value <<
          std::endl;
      }
    }
  }
}