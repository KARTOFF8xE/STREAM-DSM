#include <iostream>

#include <ipc/ipc-client.hpp>

#include <chrono>
#include <thread>

int main() {
  std::cout << "Make a Node Request..." << std::endl;
  
  IpcClient client(4);
  
  std::cout << ">> ";
  primaryKey_t pKey;
  std::cin >> pKey;
  requestId_t requestId;
  const NodeRequest request{
    .primaryKey = pKey,
    .updates = true
  };
  client.sendNodeRequest(request, requestId, true);
  
  while (true) {
    {
      std::optional<NodeResponse> optResp = client.receiveNodeResponse(false);
      if (optResp.has_value()) {
        NodeResponse resp = optResp.value();
        std::cout <<
          "Received Node Reponse" <<
          "\n\tprimaryKey: " << resp.primaryKey <<
          "\n\tname: " << resp.name <<
          "\n\tpid: " << resp.pid <<
          "\n\talive: " << resp.alive <<
          "\n\taliveChangeTime: " << resp.aliveChangeTime <<
          "\n\tbootCount: " << resp.bootCount <<
          "\n\tnrOfNeighours: " << resp.nrOfInitialUpdates <<
          std::endl;
      }
    }
    {
     std::optional<NodePublishersToUpdate> optResp = client.receiveNodePublishersToUpdate(false);
      if (optResp.has_value()) {
        NodePublishersToUpdate resp = optResp.value();
        std::cout << "Received PublishersUpdate" <<
          "\n\tprimaryKey: " << resp.primaryKey <<
          "\n\tpublishesTo: " << resp.publishesTo <<
          "\n\tisUpdate: " << resp.isUpdate <<
          std::endl;
      }
    }
    {
     std::optional<NodeSubscribersToUpdate> optResp = client.receiveNodeSubscribersToUpdate(false);
      if (optResp.has_value()) {
        NodeSubscribersToUpdate resp = optResp.value();
        std::cout << "Received SubscribersUpdate" <<
          "\n\tprimaryKey: " << resp.primaryKey <<
          "\n\tsubscribesTo: " << resp.subscribesTo <<
          "\n\tisUpdate: " << resp.isUpdate <<
          std::endl;
      }
    }
    {
     std::optional<NodeIsClientOfUpdate> optResp = client.receiveNodeIsClientOfUpdate(false);
      if (optResp.has_value()) {
        NodeIsClientOfUpdate resp = optResp.value();
        std::cout << "Received isClientToUpdate" <<
          "\n\tprimaryKey: " << resp.primaryKey <<
          "\n\tsrvName: " << resp.srvName <<
          "\n\tserverNodeId: " << resp.serverNodeId <<
          "\n\tisUpdate: " << resp.isUpdate <<
          std::endl;
      }
    }
    {
     std::optional<NodeIsServerForUpdate> optResp = client.receiveNodeIsServerForUpdate(false);
      if (optResp.has_value()) {
        NodeIsServerForUpdate resp = optResp.value();
        std::cout << "Received isServerForUpdate" <<
          "\n\tprimaryKey: " << resp.primaryKey <<
          "\n\tsrvName: " << resp.srvName <<
          "\n\tclientNodeId: " << resp.clientNodeId <<
          "\n\tisUpdate: " << resp.isUpdate <<
          std::endl;
      }
    }
    {
     std::optional<NodeIsActionClientOfUpdate> optResp = client.receiveNodeIsActionClientOfUpdate(false);
      if (optResp.has_value()) {
        NodeIsActionClientOfUpdate resp = optResp.value();
        std::cout << "Received isActionClientToUpdate" <<
          "\n\tprimaryKey: " << resp.primaryKey <<
          "\n\tsrvName: " << resp.srvName <<
          "\n\tactionserverNodeId: " << resp.actionserverNodeId <<
          "\n\tisUpdate: " << resp.isUpdate <<
          std::endl;
      }
    }
    {
     std::optional<NodeIsActionServerForUpdate> optResp = client.receiveNodeIsActionServerForUpdate(false);
      if (optResp.has_value()) {
        NodeIsActionServerForUpdate resp = optResp.value();
        std::cout << "Received isActionServerForUpdate" <<
          "\n\tprimaryKey: " << resp.primaryKey <<
          "\n\tsrvName: " << resp.srvName <<
          "\n\tactionclientNodeId: " << resp.actionclientNodeId <<
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