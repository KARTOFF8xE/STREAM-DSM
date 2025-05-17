#include <iostream>

#include <ipc/ipc-client.hpp>

#include <chrono>
#include <thread>

int main() {
  std::cout << "Make a Single Attribute Request..." << std::endl;
  
  IpcClient client(4);
  
  std::cout << ">> ";
  primaryKey_t pKey;
  std::cin >> pKey;
  requestId_t requestId;
  const SingleAttributesRequest request{
    .primaryKey = pKey,
    .attribute = CPU_UTILIZATION,
    .continuous = true,
  };
  std::cout << "send request..." << std::flush;
  client.sendSingleAttributesRequest(request, requestId, false);
  std::cout << "done" << std::endl;
  while (true) {
    {
      std::optional<SingleAttributesResponse> optResp = client.receiveSingleAttributesResponse(false);
      if (optResp.has_value()) {
        SingleAttributesResponse resp = optResp.value();
        std::cout <<
          "Received Node Reponse:" <<
          "\n\tvalue: " << resp.value <<
          std::endl;
      }
    }
  }
}