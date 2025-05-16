#include <iostream>

#include <ipc/ipc-client.hpp>
#include <ipc/util.hpp>

#include <chrono>
#include <thread>
#include <vector>
#include <bits/stdc++.h>

int main() {
  std::cout << "Make a Custom Attribute Request..." << std::endl;
  
  IpcClient client(4);

  std::string query = R"(from(bucket: "STREAM")
  |> range(start: 0)
  |> filter(fn: (r) => r["_measurement"] == "CPU_UTILIZATION")
  |> filter(fn: (r) => r["_field"] == "value")
  |> filter(fn: (r) => r["primaryKey"] == "4")
  |> mean())";

  requestId_t requestId;
  std::stringstream ss(query);
  std::string line;
  std::vector<std::string> lines;
  while (std::getline(ss, line, '\n')) {
    lines.push_back(line);
  }
    
  CustomAttributesRequest request {
    .continuous = true,
  };
  util::parseStringArray(request.query, lines);

  std::cout << "send request..." << std::flush;
  client.sendCustomAttributesRequest(request, requestId, false);
  std::cout << "done" << std::endl;

  while (true) {
    {
      std::optional<CustomAttributesResponse> optResp = client.receiveCustomAttributesResponse(false);
      if (optResp.has_value()) {
        CustomAttributesResponse payload = optResp.value();
        std::cout << payload.number << ": " << payload.line << std::endl;
      }
    }
  }
}