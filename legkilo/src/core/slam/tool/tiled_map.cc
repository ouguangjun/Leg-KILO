// SPDX-License-Identifier: MIT
// @file tiled_map.cc
// @brief Tiled map utility implementation.
// @author Ou Guangjun
// @created 2026-04-05
// @maintainer ouguangjun98@gmail.com

#include "core/slam/tool/tiled_map.h"

#include <algorithm>
#include <fstream>
#include <stdexcept>

#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>

#include "common/file_io.hpp"

namespace legkilo {

namespace {

std::string joinPath(const std::string& dir, const std::string& name) {
    if (dir.empty()) return name;
    if (dir.back() == '/') return dir + name;
    return dir + "/" + name;
}

}  // namespace

void TiledMap::setOutputFolder(const std::string& folder_path) {
    output_folder_ = folder_path;
    file_io::ensureDirectory(output_folder_);
    file_io::ensureDirectory(cloudsFolder());
}

TiledMap::TilePtr TiledMap::findTile(const TileKey& key) const {
    const auto it = tiles_.find(key);
    return it == tiles_.end() ? nullptr : it->second;
}

TiledMap::TilePtr TiledMap::getOrCreateTile(const TileKey& key) {
    const auto it = tiles_.find(key);
    if (it != tiles_.end()) return it->second;

    TilePtr tile = std::make_shared<Tile>();
    tile->key = key;
    tile->relative_path = makeRelativeTilePath(key);
    tile->absolute_path = joinPath(output_folder_, tile->relative_path);
    tiles_.insert({key, tile});
    return tile;
}

void TiledMap::insertPoints(const TileKey& key, const CloudPtr& cloud) {
    if (!cloud || cloud->empty()) return;

    const TilePtr tile = getOrCreateTile(key);
    if (!tile->cloud) {
        if (!loadTile(tile)) { tile->cloud = pcl_utils::makeCloud<PointType>(); }
    }

    *tile->cloud += *cloud;
    tile->point_num = tile->cloud->size();
    tile->dirty = true;
}

bool TiledMap::loadTile(const TilePtr& tile) {
    if (!tile) return false;
    if (tile->cloud) return true;

    tile->cloud = pcl_utils::makeCloud<PointType>();
    if (!boost::filesystem::exists(tile->absolute_path)) {
        tile->point_num = 0;
        return true;
    }

    if (pcl::io::loadPCDFile(tile->absolute_path, *tile->cloud) != 0) {
        tile->cloud.reset();
        return false;
    }

    tile->point_num = tile->cloud->size();
    return true;
}

bool TiledMap::flushTile(const TilePtr& tile) {
    if (!tile) return false;
    if (!tile->cloud) return true;
    if (!tile->dirty) return true;

    file_io::ensureDirectory(cloudsFolder());
    tile->point_num = tile->cloud->size();
    const int ret = pcl::io::savePCDFileBinaryCompressed(tile->absolute_path, *tile->cloud);
    if (ret != 0) return false;
    tile->dirty = false;
    return true;
}

void TiledMap::releaseTile(const TilePtr& tile) {
    if (!tile || !tile->cloud) return;
    tile->point_num = tile->cloud->size();
    tile->cloud.reset();
}

void TiledMap::flushAll() {
    for (const auto& [key, tile] : tiles_) { flushTile(tile); }
}

void TiledMap::writeIndex(const std::string& filepath) const {
    std::vector<TilePtr> tiles = getAllTiles();
    std::sort(tiles.begin(), tiles.end(), [](const TilePtr& lhs, const TilePtr& rhs) {
        if (lhs->key.ix != rhs->key.ix) return lhs->key.ix < rhs->key.ix;
        return lhs->key.iy < rhs->key.iy;
    });

    std::ofstream ofs(filepath, std::ios::out | std::ios::trunc);
    if (!ofs.is_open()) { throw std::runtime_error("Failed to write tile index: " + filepath); }

    ofs << "tile_id,tile_ix,tile_iy,center_x,center_y,min_x,max_x,min_y,max_y,point_num,pcd_path\n";

    size_t tile_id = 0;
    for (const auto& tile : tiles) {
        const double min_x = tile_size_ * static_cast<double>(tile->key.ix);
        const double min_y = tile_size_ * static_cast<double>(tile->key.iy);
        const double max_x = min_x + tile_size_;
        const double max_y = min_y + tile_size_;
        const double center_x = 0.5 * (min_x + max_x);
        const double center_y = 0.5 * (min_y + max_y);
        ofs << tile_id++ << "," << tile->key.ix << "," << tile->key.iy << "," << center_x << "," << center_y << ","
            << min_x << "," << max_x << "," << min_y << "," << max_y << "," << tile->point_num << ","
            << tile->relative_path << "\n";
    }
}

std::vector<TiledMap::TilePtr> TiledMap::getAllTiles() const {
    std::vector<TilePtr> tiles;
    tiles.reserve(tiles_.size());
    for (const auto& [key, tile] : tiles_) { tiles.push_back(tile); }
    return tiles;
}

std::string TiledMap::cloudsFolder() const { return joinPath(output_folder_, "clouds"); }

std::string TiledMap::makeRelativeTilePath(const TileKey& key) const {
    return "clouds/tile_" + std::to_string(key.ix) + "_" + std::to_string(key.iy) + ".pcd";
}

}  // namespace legkilo
