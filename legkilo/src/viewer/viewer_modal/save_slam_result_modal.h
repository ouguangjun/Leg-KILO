
// SPDX-License-Identifier: MIT
// @file save_slam_result_modal.h
// @brief Modal for saving SLAM result in the viewer.
// @author Ou Guangjun
// @created 2026-03-29
// @maintainer ouguangjun98@gmail.com

#ifndef LEGKILO_SAVE_SLAM_RESULT_MODAL_H
#define LEGKILO_SAVE_SLAM_RESULT_MODAL_H

#include <atomic>
#include <memory>
#include <string>

#include "core/slam/tool/slam_result_saver.h"

namespace guik {
class ProgressModal;
}

namespace legkilo {

class SaveSLAMResultModal {
   public:
    SaveSLAMResultModal();

    ~SaveSLAMResultModal() = default;

    void setInputPath(const std::string& input_path) { save_input_path_ = input_path; }

    void setOutputPath(const std::string& output_path) { save_output_path_ = output_path; }

    void RequestToSave() { request_to_save_.store(true); }

    bool run();

   private:
    void resetSetting();

   private:
    std::atomic<bool> request_to_save_ = false;

    std::string save_input_path_ = "";

    std::string save_output_path_ = "";

    std::unique_ptr<guik::ProgressModal> progress_modal_ = nullptr;

    // SLAM result saver
    std::unique_ptr<SLAMResultSaver> slam_result_saver_ = nullptr;

    // trajectory saving options
    bool save_traj_ = false;
    int traj_coord_type_ = 0;
    int traj_format_ = 0;
    int traj_type_ = 2;

    // global map saving options
    bool save_global_map_ = false;
    int global_map_coord_type_ = 0;
    int global_map_size_type_ = 0;
    float global_map_tile_size_ = 50.0f;
    bool global_map_downsample_enable_ = false;
    float global_map_voxel_resolution_ = 0.2f;

    // slam running result saving options
    bool save_slam_running_result_ = false;
};
}  // namespace legkilo
#endif  // LEGKILO_SAVE_SLAM_RESULT_MODAL_H
