#pragma once

#include <ipc/sharedMem.hpp>
#include <ipc/ipc-client.hpp>

template<typename T>
void printResponse(const sharedMem::Response& response) {
    std::cerr << "No specialization of printResponse for the requested type.\n";
}
template<>
void printResponse<sharedMem::NumericalResponse>(const sharedMem::Response& response);
template<>
void printResponse<sharedMem::TextualResponse>(const sharedMem::Response& response);

template<>
void printResponse<sharedMem::NumericalResponse>(const sharedMem::Response& response) {
    if (response.header.type != sharedMem::ResponseType::NUMERICAL) {
        std::cerr << "Error: Response is not of type NUMERICAL\n";
        return;
    }

    const sharedMem::NumericalResponse& nr = response.numerical;
    std::cout << "NumericalResponse:\tNumber: " << nr.number << "  Total: " << nr.total << "  Value: " << nr.value << std::endl;
}

template<>
void printResponse<sharedMem::TextualResponse>(const sharedMem::Response& response) {
    if (response.header.type != sharedMem::ResponseType::TEXTUAL) {
        std::cerr << "Error: Response is not of type TEXTUAL\n";
        return;
    }

    const sharedMem::TextualResponse& tr = response.textual;
    std::cout << "TextualResponse:\tNumber: " << tr.number << "  Total: " << tr.total << "  Value: " << tr.line << std::endl;
}