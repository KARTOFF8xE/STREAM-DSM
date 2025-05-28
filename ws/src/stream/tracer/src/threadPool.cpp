#include "threadPool.hpp"

#include "babeltrace.hpp"
#include <iostream>


ThreadPool::ThreadPool(size_t num_workers) : stopFlag(false) {
    for (size_t i = 0; i < num_workers; ++i) {
        workers.emplace_back(&ThreadPool::workerThread, this, i);
    }
}

ThreadPool::~ThreadPool() {
    {
        std::unique_lock<std::mutex> lock(mtx);
        stopFlag = true;
    }
    cv.notify_all();

    for (auto& w : workers) {
        if (w.joinable()) {
            w.join();
        }
    }
}

void ThreadPool::enqueue(std::string task) {
    {
        std::unique_lock<std::mutex> lock(mtx);
        tasks.push(std::move(task));
    }
    cv.notify_one();
}

void ThreadPool::workerThread(size_t worker_id) {
    while (true) {
        std::string task;
        {
            std::unique_lock<std::mutex> lock(mtx);
            cv.wait(lock, [this]() { return stopFlag || !tasks.empty(); });
            if (stopFlag && tasks.empty()) {
                return;
            }
            task = std::move(tasks.front());
            tasks.pop();
        }
        std::cout << "\tWorker " << worker_id << "\tpath: " << task << std::endl;
        createAndExecuteTraceGraph(task.c_str());
    }
}
