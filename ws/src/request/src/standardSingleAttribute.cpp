#include <iostream>

#include <ipc/ipc-client.hpp>

#include <chrono>
#include <thread>

int main() {
  std::cout << "Make a Standard Single Attribute Request..." << std::endl;
  
  IpcClient client(4);
  
  std::cout << ">> ";
  primaryKey_t pKey;
  std::cin >> pKey;
  requestId_t requestId;
  const StandardSingleAttributesRequest request{
    .primaryKey = pKey,
    .attribute = CPU_UTILIZATION,
    .continuous = false,
  };
  std::cout << "send request..." << std::flush;
  client.sendStandardSingleAttributesRequest(request, requestId, false);
  std::cout << "done" << std::endl;
  while (true) {
    {
      std::optional<StandardSingleAttributesResponse> optResp = client.receiveStandardSingleAttributesResponse(false);
      if (optResp.has_value()) {
        StandardSingleAttributesResponse resp = optResp.value();
        std::cout <<
          "Received Node Reponse" <<
          "\n\tprimaryKey: " << resp.primaryKey <<
          "\n\tvalue: " << resp.value <<
          std::endl;
      }
      // std::cout << "..." << std::endl;
    }
  }
}