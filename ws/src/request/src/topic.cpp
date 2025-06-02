#include <iostream>

#include <ipc/ipc-client.hpp>
#include <ipc/util.hpp>

#include <chrono>
#include <thread>

int main() {
  std::cout << "Make a Topic Request..." << std::endl;
  
  IpcClient client(1);
  
  std::cout << ">> ";
  std::string pKey;
  std::cin >> pKey;
  requestId_t requestId;
  TopicRequest request{ .updates = true };
  util::parseString(request.primaryKey, pKey);
  client.sendTopicRequest(request, requestId, true);

  while (true) {
    {
      std::optional<TopicResponse> optResp = client.receiveTopicResponse(false);
      if (optResp.has_value()) {
        TopicResponse resp = optResp.value();
        std::cout <<
          "Received Node Reponse:" <<
          "\n\tprimaryKey: " << resp.primaryKey <<
          "\n\tname: " << resp.name <<
          "\n\tnrOfNeighours: " << resp.nrOfInitialUpdates <<
          std::endl;
      }
    }
    {
     std::optional<TopicPublishersUpdate> optResp = client.receiveTopicPublishersUpdate(false);
      if (optResp.has_value()) {
        TopicPublishersUpdate resp = optResp.value();
        std::cout << "Received PublishersUpdate" <<
          "\n\tprimaryKey: " << resp.primaryKey <<
          "\n\tpublisher: " << resp.publisher <<
          "\n\tisUpdate: " << resp.isUpdate <<
          std::endl;
      }
    }
    {
     std::optional<TopicSubscribersUpdate> optResp = client.receiveTopicSubscribersUpdate(false);
      if (optResp.has_value()) {
        TopicSubscribersUpdate resp = optResp.value();
        std::cout << "Received SubscribersUpdate" <<
          "\n\tprimaryKey: " << resp.primaryKey <<
          "\n\tsubscriber: " << resp.subscriber <<
          "\n\tisUpdate: " << resp.isUpdate <<
          std::endl;
      }
    }
  }
  

  // std::this_thread::sleep_for(std::chrono::milliseconds(500));
  // const ProcessRequest request2{
  //   .primaryKey = (pid_t)14,
  //   .updates = false,
  //   .continuous = false,
  // };  
  // std::cout << "Hello Primary 2!" << std::endl;
  // client.sendProcessRequest(request2, requestId, true);

  // std::this_thread::sleep_for(std::chrono::milliseconds(500));
  // const ProcessRequest request3{
  //   .primaryKey = (pid_t)15,
  //   .updates = true,
  //   .continuous = false,
  // };  
  // std::cout << "Hello Primary 3!" << std::endl;
  // client.sendProcessRequest(request3, requestId, true);

  // std::this_thread::sleep_for(std::chrono::milliseconds(500));
  // const ProcessRequest request4{
  //   .primaryKey = (pid_t)13,
  //   .updates = false,
  //   .continuous = false,
  // };  
  // std::cout << "Hello Primary 4!" << std::endl;
  // client.sendProcessRequest(request4, requestId, true);

  // std::this_thread::sleep_for(std::chrono::milliseconds(500));
  // const ProcessRequest request5{
  //   .primaryKey = (pid_t)15,
  //   .updates = false,
  //   .continuous = false,
  // };  
  // std::cout << "Hello Primary 5!" << std::endl;
  // client.sendProcessRequest(request5, requestId, true);
}