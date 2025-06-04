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
          "Received Topic Reponse:" <<
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
          "\n\tedge: "      << resp.edge <<
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
          "\n\tedge: "      << resp.edge <<
          "\n\tisUpdate: " << resp.isUpdate <<
          std::endl;
      }
    }
    sleep(1);
  }
}