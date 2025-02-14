#include <iostream>

#include <ipc/ipc-client.hpp>

#include <chrono>
#include <thread>

int main() {
  
  IpcClient client(2);

  requestId_t requestId;
  const ProcessRequest request{
    .primaryKey = (pid_t)13,
    .updates = true,
    .continuous = false,
  };
  std::cout << "Hello Primary 1!" << std::endl;
  client.sendProcessRequest(request, requestId, true);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  const ProcessRequest request2{
    .primaryKey = (pid_t)14,
    .updates = false,
    .continuous = false,
  };  
  std::cout << "Hello Primary 2!" << std::endl;
  client.sendProcessRequest(request2, requestId, true);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  const ProcessRequest request3{
    .primaryKey = (pid_t)15,
    .updates = true,
    .continuous = false,
  };  
  std::cout << "Hello Primary 3!" << std::endl;
  client.sendProcessRequest(request3, requestId, true);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  const ProcessRequest request4{
    .primaryKey = (pid_t)13,
    .updates = false,
    .continuous = false,
  };  
  std::cout << "Hello Primary 4!" << std::endl;
  client.sendProcessRequest(request4, requestId, true);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  const ProcessRequest request5{
    .primaryKey = (pid_t)15,
    .updates = false,
    .continuous = false,
  };  
  std::cout << "Hello Primary 5!" << std::endl;
  client.sendProcessRequest(request5, requestId, true);
}