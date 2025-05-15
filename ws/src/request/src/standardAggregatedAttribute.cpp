#include <iostream>

#include <ipc/ipc-client.hpp>

#include <chrono>
#include <thread>

int main() {
  std::cout << "Make a Standard Single Attribute Request..." << std::endl;

  IpcClient client(4);

  // std::cout << ">> ";
  // primaryKey_t pKey;
  // std::cin >> pKey;
  requestId_t requestId;
  const StandardAggregatedAttributesRequest request{
    .primaryKey_RootTree1 = 4,
    .primaryKey_RootTree2 = 11,
    .tree1 = PROCESSDRIVEN,
    .tree2 = PROCESSDRIVEN,
    .attribute = CPU_UTILIZATION,
    .binOperation = DIFFERENCE,
    .continuous = false,
  };

  std::cout << "send request..." << std::flush;
  client.sendStandardAggregatedAttributesRequest(request, requestId, false);
  std::cout << "done" << std::endl;
  while (true) {
    {
      std::optional<StandardAggregatedAttributesResponse> optResp = client.receiveStandardAggregatedAttributesResponse(false);
      if (optResp.has_value()) {
        StandardAggregatedAttributesResponse resp = optResp.value();
        std::cout <<
          "Received Node Reponse" <<
          "\n\tvalue: " << resp.value <<
          std::endl;
      }
    }
  }
}