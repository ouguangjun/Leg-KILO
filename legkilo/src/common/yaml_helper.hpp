// SPDX-License-Identifier: MIT
// @file yaml_helper.hpp
// @brief Safe YAML getters with logging and defaults.
// @author Ou Guangjun
// @created 2024-12-17
// @maintainer ouguangjun98@gmail.com
#ifndef LEG_KILO_YAML_HELPER_H
#define LEG_KILO_YAML_HELPER_H

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace legkilo {

template <typename T>
inline std::string toLogString(const T& v) {
    std::ostringstream ss;
    ss << v;
    return ss.str();
}

inline std::string toLogString(bool v) { return v ? "true" : "false"; }

template <typename T>
inline std::string toLogString(const std::vector<T>& vec) {
    std::ostringstream ss;
    ss << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        ss << toLogString(vec[i]);
        if (i != vec.size() - 1) ss << ", ";
    }
    ss << "]";
    return ss.str();
}

class YamlHelper {
   public:
    YamlHelper() = delete;

    explicit YamlHelper(const std::string& config_file) {
        try {
            yaml_node_ = YAML::LoadFile(config_file);
        } catch (const std::exception& e) {
            LOG(ERROR) << "Failed to open YAML file: " << config_file << "Errors: " << e.what();
            throw std::runtime_error("Failed to open YAML file: " + config_file);
        }
    }

    bool hasKey(const std::string& key) const { return yaml_node_[key] ? true : false; }

    template <typename T>
    T get(const std::string& key) const {
        if (!hasKey(key)) {
            LOG(ERROR) << "Failed to find key: " << key;
            throw std::runtime_error("Failed to find key: " + key);
        }
        try {
            T ret = yaml_node_[key].as<T>();
            LOG(INFO) << "YAML Key:  " << key << " = " << toLogString(ret);
            return ret;
        } catch (const std::exception& e) {
            LOG(ERROR) << "Failed to convert key " << key << "Errors: " << e.what();
            throw std::runtime_error("Failed to convert key " + key);
        }
    }

    template <typename T>
    T get(const std::string& key, const T& default_value) const {
        if (!hasKey(key)) {
            LOG(WARNING) << "Key not found: " << key << ", returning default: " << toLogString(default_value);
            return default_value;
        }
        try {
            T ret = yaml_node_[key].as<T>();
            LOG(INFO) << "YAML Key: " << key << " = " << toLogString(ret);
            return ret;
        } catch (const std::exception& e) {
            LOG(WARNING) << "Failed to convert key " << key << ", returning default: " << toLogString(default_value)
                         << ". Error: " << e.what();
            return default_value;
        }
    }

   private:
    YAML::Node yaml_node_;
};

}  // namespace legkilo
#endif  // LEG_KILO_YAML_HELPER_H
