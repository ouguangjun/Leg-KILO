#ifndef LEG_KILO_YAML_HELPER_H
#define LEG_KILO_YAML_HELPER_H

#include <iostream>
#include <sstream>
#include <string>
#include <yaml-cpp/yaml.h>
#include <glog/logging.h>

template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& vec) {
    os << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        os << vec[i];
        if (i != vec.size() - 1)
            os << ", ";
    }
    os << "]";
    return os;
}

namespace legkilo{

class YamlHelper {
public:
    YamlHelper() = delete;

    explicit YamlHelper(const std::string& config_file){
        try{
            yaml_node_ = YAML::LoadFile(config_file);
        } catch(const std::exception& e){
            LOG(ERROR) << "Failed to open YAML file: " << config_file 
                                       << "Errors: " << e.what(); 
            throw std::runtime_error("Failed to open YAML file: " + config_file);
        }
    }

    bool hasKey(const std::string& key) const{
        return yaml_node_[key] ? true : false;
    }

    template <typename T>
    T get(const std::string& key) const{
        if (!hasKey(key)) {
            LOG(ERROR) << "Failed to find key: " << key;
            throw std::runtime_error("Failed to find key: " + key);
        }
        try {
            T ret = yaml_node_[key].as<T>();
            LOG(INFO) << "YAML Key:  " << key << " = " << ret;
            return ret;
        } catch (const std::exception& e) {
            LOG(ERROR) << "Failed to convert key " << key 
                                       << "Errors: " << e.what();
            throw std::runtime_error("Failed to convert key " + key);
        }
    }
    
    template <typename T>
    T get(const std::string& key, const T& default_value) const {
        if (!hasKey(key)) {
            LOG(WARNING) << "Key not found: " << key << ", returning default: " << default_value;
            return default_value;
        }
        try {
            T ret = yaml_node_[key].as<T>();
            LOG(INFO) << "YAML Key: " << key << " = " << ret;
            return ret;
        } catch (const std::exception& e) {
            LOG(WARNING) << "Failed to convert key " << key << ", returning default: " << default_value
                        << ". Error: " << e.what();
            return default_value;
        }
    }


private:
    YAML::Node yaml_node_; 
};

} //namespace legkilo
#endif //LEG_KILO_YAML_HELPER_H