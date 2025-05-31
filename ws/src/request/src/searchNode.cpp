#include <iostream>

#include <ipc/ipc-client.hpp>
#include <ipc/util.hpp>

#include <chrono>
#include <thread>

int main() {
  std::cout << "Make a Node Search Request..." << std::endl;
  
  IpcClient client(1);
  
  std::cout << ">> ";
  std::string nodeName;
  std::cin >> nodeName;
  requestId_t requestId;
  SearchRequest request {
    .type = SearchRequest::NODE
  };
  util::parseString(request.name, nodeName);

  client.sendSearchRequest(request, requestId, true);

  std::optional<SearchResponse> optResp = client.receiveSearchResponse(true);
  if (optResp.has_value()) {
    SearchResponse resp = optResp.value();
    std::cout <<
      "Received Node Reponse" <<
      "\n\tprimaryKey: " << resp.primaryKey << std::endl;
  }
}