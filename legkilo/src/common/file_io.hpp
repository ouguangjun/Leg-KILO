// SPDX-License-Identifier: MIT
// @file file_io.hpp
// @brief File I/O operations
// @author Ou Guangjun
// @created 2026-01-31
// @maintainer ouguangjun98@gmail.com

#ifndef LEG_KILO_FILE_IO_HPP
#define LEG_KILO_FILE_IO_HPP

#include <stdexcept>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/system/error_code.hpp>

namespace legkilo {
namespace file_io {

inline bool ensureDirectory(const std::string& dir_path) {
    namespace fs = boost::filesystem;
    boost::system::error_code ec;
    fs::path p(dir_path);
    fs::file_status st = fs::status(p, ec);
    if (!ec && fs::exists(st)) {
        if (fs::is_directory(st)) return false;  // already exists
        throw std::runtime_error("Path exists but is not a directory: " + dir_path);
    }
    ec.clear();
    bool created = fs::create_directories(p, ec);
    if (ec) { throw std::runtime_error("Failed to create directory: " + dir_path + ", error: " + ec.message()); }
    return created;
}

inline bool removeRegularFileIfExists(const std::string& file_path) {
    namespace fs = boost::filesystem;
    if (!fs::exists(file_path)) return false;
    if (!fs::is_regular_file(file_path)) return false;
    return fs::remove(file_path);
}

inline void clearDirectoryContents(const std::string& dir_path) {
    namespace fs = boost::filesystem;
    fs::path p(dir_path);
    if (!fs::exists(p)) return;
    if (!fs::is_directory(p)) return;
    for (fs::directory_iterator iter(p), end; iter != end; ++iter) { fs::remove_all(iter->path()); }
}

}  // namespace file_io
}  // namespace legkilo

#endif  // LEG_KILO_FILE_IO_HPP