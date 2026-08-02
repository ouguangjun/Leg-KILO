#include "viewer/viewer_base.h"

namespace legkilo {

ViewerBase::~ViewerBase() { this->stop(); }

void ViewerBase::start() {
    request_to_terminate_.store(false);
    kill_switch_.store(false);
    worker_thread_ = std::thread(&ViewerBase::workerLoop, this);
}

void ViewerBase::stop() {
    request_to_terminate_.store(true);
    kill_switch_.store(true);
    if (!worker_thread_.joinable()) return;
    if (worker_thread_.get_id() == std::this_thread::get_id()) return;
    worker_thread_.join();
}

void ViewerBase::wait() {
    while (!request_to_terminate_.load()) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }

    this->stop();
}

void ViewerBase::invoke(const std::function<void()>& func) {
    if (kill_switch_.load()) return;
    std::lock_guard<std::mutex> lock(task_mutex_);
    task_deque_.push_back(func);
}
}  // namespace legkilo