// SPDX-License-Identifier: MIT
// @file timer_utils.hpp
// @brief Scoped timers and average latency logging helpers.
// It is not intended for very short code paths, since locking and bookkeeping can outweigh the measured time.
// @author Ou Guangjun
// @created 2024-12-17
// @maintainer ouguangjun98@gmail.com
#ifndef LEG_KILO_TIMER_UTILS_H
#define LEG_KILO_TIMER_UTILS_H

#include <glog/logging.h>
#include <chrono>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace legkilo {

class Timer {
   public:
    template <typename Func, typename... Args>
    static void measure(const std::string& func_name, Func&& func, Args&&... args) {
        getInstance().internalMeasure(func_name, std::forward<Func>(func), std::forward<Args>(args)...);
    }

    static void logAllAverTime() { getInstance().internalLogAllAverTime(); }

   private:
    struct TimeStat {
        int count = 0;
        double total_time = 0.0;
    };

    Timer() = default;
    ~Timer() = default;

    static Timer& getInstance() {
        static Timer instance;
        return instance;
    }

    template <typename Func, typename... Args>
    void internalMeasure(const std::string& func_name, Func&& func, Args&&... args) {
        auto start = std::chrono::high_resolution_clock::now();
        std::forward<Func>(func)(std::forward<Args>(args)...);
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = end - start;

        std::lock_guard<std::mutex> lock(mutex_);
        auto iter = time_stats_.find(func_name);
        if (iter != time_stats_.end()) {
            iter->second.count++;
            iter->second.total_time += duration.count();
        } else {
            keys_.push_back(func_name);
            time_stats_[func_name] = {1, duration.count()};
        }
    }

    void internalLogAllAverTime() {
        std::lock_guard<std::mutex> lock(mutex_);
        LOG(INFO) << "===== Every Component Average Time =====";
        for (const auto& key : keys_) {
            const auto iter = time_stats_.find(key);
            if (iter == time_stats_.end() || iter->second.count == 0) continue;
            LOG(INFO) << key << " average time : " << iter->second.total_time / iter->second.count << " ms";
        }
        LOG(INFO) << "======================================";
    }

    std::unordered_map<std::string, TimeStat> time_stats_;
    std::vector<std::string> keys_;  // for order
    std::mutex mutex_;
};

}  // namespace legkilo

#endif
