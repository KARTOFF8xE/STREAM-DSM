#include <iostream>

#include <ipc/ipc-client.hpp>
#include <ipc/sharedMem.hpp>
#include <ipc/util.hpp>

#include <chrono>
#include <thread>
#include <cstddef>

int main() {
  std::cout << "Make a Aggregated Attribute Request..." << std::endl;

  IpcClient client(5);

  std::cout << ">> ";
  primaryKey_t pKey;
  std::cin >> pKey;
  requestId_t requestId;
  const SHMAddressRequest request{
    .primaryKey = pKey
  };

  std::cout << "send request..." << std::flush;
  client.sendSHMAddressRequest(request, requestId, false);
  std::cout << "done" << std::endl;
  std::optional<SHMAddressResponse> response = client.receiveSHMAddressResponse();
  if (!response.has_value()) {
    std::cerr << "no response" << std::endl;
    return 1;
  }
  
  SHMAddressResponse payload = response.value();
  std::cout << "sharedMem Address: " << payload.memAdress << std::endl;
  
  sharedMem::SHMChannel channel(util::parseString(payload.memAdress).c_str(), false);
  std::cout << util::parseString(payload.memAdress) << std::endl;
  double counter = 0;
  while (true) {
    sharedMem::Value msg {
      .timestamp  = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()),
      .primaryKey = pKey,
      .value      = counter++,
      .type       = sharedMem::MessageType::DISK,
    };
    if (counter > 1000) counter = 0;
    channel.send(msg);
    // std::cout << "send " << msg.value << std::endl;
    // usleep(1000);
  }
}