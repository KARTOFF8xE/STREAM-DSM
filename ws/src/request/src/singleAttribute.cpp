#include <iostream>

#include "common.hpp"

#include <ipc/ipc-client.hpp>
#include <ipc/sharedMem.hpp>

#include <chrono>
#include <thread>

int main() {
  std::cout << "Make a Single Attribute Request..." << std::endl;
  
  IpcClient client(4);
  
  std::cout << ">> ";
  primaryKey_t pKey;
  std::cin >> pKey;
  requestId_t requestId;
  const SingleAttributesRequest request {
    .primaryKey = pKey,
    .attribute = PUBLISHINGRATES,
    .direction = Direction::NONE,
    .continuous = true,
  };
  std::cout << "send request..." << std::flush;
  client.sendSingleAttributesRequest(request, requestId, false);
  std::cout << "done" << std::endl;
  std::optional<SingleAttributesResponse> response = client.receiveSingleAttributesResponse();
  if (!response.has_value()) { std::cerr << "Response got no Value" << std::flush; return 1; }
  SingleAttributesResponse resp = response.value();

  sharedMem::SHMChannel<sharedMem::Response> channel(resp.memAddress, true);

  while (true) {
    {
      sharedMem::Response sharedMemResponse {};
      if (!channel.receive(sharedMemResponse, false)) continue;

      printResponse<sharedMem::NumericalResponse>(sharedMemResponse);
    }
  }
}