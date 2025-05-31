#include <iostream>

#include "common.hpp"

#include <ipc/ipc-client.hpp>
#include <ipc/sharedMem.hpp>
#include <ipc/util.hpp>

#include <chrono>
#include <thread>
#include <vector>
#include <bits/stdc++.h>

std::vector<std::string> getCustomResponseQuery(std::string str) {
    std::vector<std::string> splitted;
    
    for (std::size_t i = 0; i < str.length(); i += MAX_STRING_SIZE) {
        splitted.push_back(str.substr(i, MAX_STRING_SIZE));
    }
    
    return splitted;
}

int main() {
  std::cout << "Make a Custom Member Request..." << std::endl;
  
  IpcClient client(4);

  std::string query = "MATCH (n) RETURN n ";

  requestId_t requestId;
  std::vector<std::string> lines = getCustomResponseQuery(query);
    
  CustomMemberRequest request {
    .continuous = true,
  };
  util::parseStringArray(request.query, lines);

  std::cout << "send request..." << std::flush;
  client.sendCustomMemberRequest(request, requestId, false);
  std::cout << "done" << std::endl;
  std::optional<CustomMemberResponse> response = client.receiveCustomMemberResponse();
  if (!response.has_value()) { std::cerr << "Response got no Value" << std::flush; return 1; }
  CustomMemberResponse resp = response.value();

  sharedMem::SHMChannel<sharedMem::Response> channel(resp.memAddress, true);

  while (true) {
    {
      sharedMem::Response sharedMemResponse {};
      if (!channel.receive(sharedMemResponse, false)) continue;

      printResponse<sharedMem::TextualResponse>(sharedMemResponse);
    }
  }
}