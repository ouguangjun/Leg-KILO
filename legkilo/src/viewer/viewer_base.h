// SPDX-License-Identifier: MIT
// @file viewer_base.h
// @brief Base class for all viewers in the application.
// @author Ou Guangjun
// @created 2026-02-07
// @maintainer ouguangjun98@gmail.com
// This file is inspired by the design of https://github.com/koide3/glim (MIT License).

#ifndef LEG_KILO_VIEWER_BASE_H
#define LEG_KILO_VIEWER_BASE_H

#include <atomic>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace legkilo {

class ViewerBase {
   public:
    ViewerBase() : request_to_terminate_(false), kill_switch_(false) {}

    virtual ~ViewerBase();

    void start();

    void wait();

    void stop();

    bool shouldTerminate() const { return request_to_terminate_.load(); }

    virtual std::string name() const = 0;

   protected:
    virtual void workerLoop() = 0;

    virtual void menuCallback() = 0;

    virtual void runModal() {};

    virtual void displayPanelCallback() {}

    void invoke(const std::function<void()>& func);

   protected:
    std::thread worker_thread_;

    std::mutex task_mutex_;
    std::deque<std::function<void()>> task_deque_;

    std::atomic_bool request_to_terminate_{false};
    std::atomic_bool kill_switch_{false};
};
}  // namespace legkilo
#endif  // LEG_KILO_VIEWER_BASE_H
