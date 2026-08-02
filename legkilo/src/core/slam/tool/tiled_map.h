// SPDX-License-Identifier: MIT
// @file tiled_map.h
// @brief Tiled map utility for global map export.
// @author Ou Guangjun
// @created 2026-04-05
// @maintainer ouguangjun98@gmail.com

#ifndef LEGKILO_TILED_MAP_H
#define LEGKILO_TILED_MAP_H

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "common/pcl_types.h"

namespace legkilo {

class TiledMap {
   public:
    struct TileKey {
        int ix = 0;
        int iy = 0;

        bool operator==(const TileKey& other) const { return ix == other.ix && iy == other.iy; }
    };

    struct TileKeyHash {
        size_t operator()(const TileKey& key) const {
            const size_t hx = std::hash<int>()(key.ix);
            const size_t hy = std::hash<int>()(key.iy);
            return hx ^ (hy + 0x9e3779b9 + (hx << 6) + (hx >> 2));
        }
    };

    struct Tile {
        TileKey key;
        std::string relative_path;
        std::string absolute_path;
        CloudPtr cloud;
        size_t point_num = 0;
        bool dirty = false;
    };

    using TilePtr = std::shared_ptr<Tile>;

   public:
    TiledMap() = default;

    void setTileSize(double size) { tile_size_ = size; }

    double getTileSize() const { return tile_size_; }

    void setOutputFolder(const std::string& folder_path);

    TilePtr findTile(const TileKey& key) const;

    TilePtr getOrCreateTile(const TileKey& key);

    void insertPoints(const TileKey& key, const CloudPtr& cloud);

    bool loadTile(const TilePtr& tile);

    bool flushTile(const TilePtr& tile);

    void releaseTile(const TilePtr& tile);

    void flushAll();

    void writeIndex(const std::string& filepath) const;

    std::vector<TilePtr> getAllTiles() const;

   private:
    std::string cloudsFolder() const;

    std::string makeRelativeTilePath(const TileKey& key) const;

   private:
    double tile_size_ = 50.0;

    std::string output_folder_;

    std::unordered_map<TileKey, TilePtr, TileKeyHash> tiles_;
};

}  // namespace legkilo

#endif  // LEGKILO_TILED_MAP_H
