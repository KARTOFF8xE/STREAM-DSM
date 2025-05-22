#include <iostream>

#include <ipc/ipc-client.hpp>
#include <ipc/sharedMem.hpp>

#include <chrono>
#include <thread>


template<typename T>
void printResponse(const sharedMem::Response& response);

template<>
void printResponse<sharedMem::NumericalResponse>(const sharedMem::Response& response) {
    if (response.header.type != sharedMem::ResponseType::NUMERICAL) {
        std::cerr << "Error: Response is not of type NUMERICAL\n";
        return;
    }

    const sharedMem::NumericalResponse& nr = response.numerical;
    std::cout << "NumericalResponse:\n";
    std::cout << "  Number: " << nr.number << "\n";
    std::cout << "  Total: " << nr.total << "\n";
    std::cout << "  Value: " << nr.value << "\n";
}

template<>
void printResponse<sharedMem::TextualResponse>(const sharedMem::Response& response) {
    if (response.header.type != sharedMem::ResponseType::TEXTUAL) {
        std::cerr << "Error: Response is not of type TEXTUAL\n";
        return;
    }

    const sharedMem::TextualResponse& tr = response.textual;
    std::cout << "TextualResponse:\n";
    std::cout << "  Number: " << tr.number << "\n";
    std::cout << "  Total: " << tr.total << "\n";
    std::cout << "  Line:   " << tr.line << "\n";
}


int main() {
  std::cout << "Make a Aggregated Attribute Request..." << std::endl;

  IpcClient client(4);

  // std::cout << ">> ";
  // primaryKey_t pKey;
  // std::cin >> pKey;
  requestId_t requestId;
  const AggregatedAttributesRequest request{
    .rootedTree1 {
      .primaryKey = 4,
      .tree       = PROCESSDRIVEN,
    },
    .rootedTree2 {
      .primaryKey = 11,
      .tree       = PROCESSDRIVEN
    },
    .attribute = CPU_UTILIZATION,
    .binOperation = DIFFERENCE,
    .continuous = true,
  };

  std::cout << "send request..." << std::flush;
  client.sendAggregatedAttributesRequest(request, requestId, false);
  std::cout << "done" << std::endl;
  std::optional<AggregatedAttributesResponse> response = client.receiveAggregatedAttributesResponse();
  if (!response.has_value()) { std::cerr << "Response got no Value" << std::flush; return 1; }
  AggregatedAttributesResponse resp = response.value();

  sharedMem::SHMChannel<sharedMem::Response> channel(resp.memAddress, true);

  while (true) {
    {
      sharedMem::Response sharedMemResponse {};
      if (!channel.receive(sharedMemResponse, false)) continue;

      sharedMem::printResponse<sharedMem::NumericalResponse>(sharedMemResponse);
    }
  }
}