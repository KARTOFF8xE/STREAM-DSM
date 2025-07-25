#include <iostream>

#include <ipc/ipc-client.hpp>
#include <ipc/sharedMem.hpp>
#include <ipc/util.hpp>

#include <chrono>
#include <thread>
#include <cstddef>

int main() {
  std::cout << "Make a Aggregated Attribute Request..." << std::endl;

  IpcClient client(1);

  std::cout << ">> ";
  std::string pKey;
  std::cin >> pKey;
  requestId_t requestId;
  SHMAddressRequest request;
  util::parseString(request.primaryKey, pKey);

  std::cout << "send request..." << std::flush;
  client.sendSHMAddressRequest(request, requestId, false);
  std::cout << "done" << std::endl;
  std::optional<SHMAddressResponse> response = client.receiveSHMAddressResponse();
  if (!response.has_value()) {
    std::cerr << "no response" << std::endl;
    return 1;
  }
  
  SHMAddressResponse payload = response.value();
  std::cout << "sharedMem Address: " << payload.memAddress << std::endl;
  
  sharedMem::SHMChannel<sharedMem::InputValue> channel(util::parseString(payload.memAddress).c_str(), false);
  std::cout << util::parseString(payload.memAddress) << std::endl;
  double counter = 0;
  while (true) {
    sharedMem::InputValue msg {
      .timestamp  = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()),
      .value      = counter++,
      .type       = sharedMem::MessageType::DISK,
    };
    util::parseString(msg.primaryKey, pKey);

    if (counter > 1000) counter = 0;
    channel.send(msg);
    // std::cout << "send " << msg.value << std::endl;
    // usleep(1000);
  }
}