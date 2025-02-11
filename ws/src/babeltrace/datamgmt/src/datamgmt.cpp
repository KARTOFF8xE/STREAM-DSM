#include <string>
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>

std::atomic<bool> runProcObserver;

void procObserver() {
    int counter = 0;
    while (runProcObserver) {
        std::cout << counter++ << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

int main() {
    runProcObserver = true;
    std::thread procObserverThread(procObserver);

    for (int i = 0; i < 15; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
        std::cout << "in main: " << i << std::endl;
    }

    runProcObserver = false;
    procObserverThread.join();
    std::cout << "finished thread" << std::endl;

}