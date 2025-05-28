#pragma once

#include <string>
#include <vector>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>


class ThreadPool {
public:
    explicit ThreadPool(size_t numWorkers);
    ~ThreadPool();

    void enqueue(std::string task);

private:
    void workerThread(size_t workerId);

    std::vector<std::thread> workers;
    std::queue<std::string> tasks;
    std::mutex mtx;
    std::condition_variable cv;
    bool stopFlag;
};