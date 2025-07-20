#ifndef LEG_KILO_TIMER_UTILS_H
#define LEG_KILO_TIMER_UTILS_H

#include <iostream>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <utility>
#include <glog/logging.h>

namespace legkilo {

class Timer {
public:
    template <typename Func, typename... Args>
    static void measure(const std::string& func_name, Func&& func, Args&&... args) {
        getInstance().internalMeasure(func_name, std::forward<Func>(func), std::forward<Args>(args)...);
    }

    static void logAllAverTime() {
        getInstance().internalLogAllAverTime();
    }

private:
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

        if (average_time_map_.find(func_name) != average_time_map_.end()) {
            auto& num = average_time_map_[func_name].first;
            auto& aver_time = average_time_map_[func_name].second;
            num++;
            aver_time = (aver_time * (num - 1) + duration.count()) / num;
        } else {
            keys_.push_back(func_name);
            average_time_map_[func_name] = {1, duration.count()};
        }
    }

    void internalLogAllAverTime() {
        LOG(INFO) << "===== Every Component Average Time =====";
        for (const auto& key : keys_) {
            LOG(INFO) << key << " average time : " 
                      << average_time_map_[key].second << " ms";
        }
        LOG(INFO) << "======================================";
    }

    std::unordered_map<std::string, std::pair<int, double>> average_time_map_;
    std::vector<std::string> keys_;  // for order
};

} // namespace legkilo

#endif
