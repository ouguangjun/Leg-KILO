#include "viewer/viewer_modal/save_slam_result_modal.h"

#include <cmath>

#include <glog/logging.h>

#include "guik/progress_modal.hpp"
#include "guik/recent_files.hpp"
#include "portable-file-dialogs.h"

namespace legkilo {

SaveSLAMResultModal::SaveSLAMResultModal() {
    progress_modal_ = std::make_unique<guik::ProgressModal>("SLAM Result Saver");
}

bool SaveSLAMResultModal::run() {
    bool save_flag = false;

    bool confirm_to_save = false;

    if (request_to_save_.load() && !save_input_path_.empty()) {
        LOG(INFO) << "Saving SLAM result from: " << save_input_path_;
        confirm_to_save = true;
    }

    request_to_save_.store(false);

    if (confirm_to_save) {
        ImGui::OpenPopup("SLAM Result Saver Modal");
        slam_result_saver_ = std::make_unique<SLAMResultSaver>();
    }

    if (ImGui::BeginPopupModal("SLAM Result Saver Modal", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(6, 4));
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8, 8));

        ImGui::TextColored(ImVec4(0.9f, 0.9f, 1.0f, 1.0f), "SLAM Result Save Configuration");
        ImGui::Separator();
        ImGui::Spacing();

        ImGui::Text("Output Path:");
        ImGui::SameLine(200);
        if (ImGui::Button("Select Output Folder", ImVec2(200, 0))) {
            guik::RecentFiles recent_files("slam_save_output_folder");
            auto result = pfd::select_folder("Select SLAM Result Save Folder", recent_files.most_recent()).result();
            if (!result.empty()) {
                save_output_path_ = result;
                recent_files.push(result);
            }
        }

        ImGui::Dummy(ImVec2(0, 5));
        ImGui::Indent(20);
        if (!save_output_path_.empty()) {
            ImGui::TextWrapped("Path: %s", save_output_path_.c_str());
        } else {
            ImGui::TextDisabled("Please select an empty folder.");
        }
        ImGui::Unindent(20);

        ImGui::Separator();

        // trajectory saving options
        static std::vector<const char*> traj_format_items = {"TUM", "KITTI"};
        static std::vector<const char*> traj_type_items = {"Frontend", "Backend", "All"};
        static std::vector<const char*> traj_coord_type_items = {"IMU", "LIDAR"};
        ImGui::Checkbox("save trajectory enable", &save_traj_);

        ImGui::BeginDisabled(!save_traj_);
        ImGui::Combo("Trajectory Coordinate Type", &traj_coord_type_, traj_coord_type_items.data(),
                     traj_coord_type_items.size());
        ImGui::Combo("Trajectory Format", &traj_format_, traj_format_items.data(), traj_format_items.size());
        ImGui::Combo("Trajectory Type", &traj_type_, traj_type_items.data(), traj_type_items.size());
        ImGui::EndDisabled();

        ImGui::Separator();

        // global map saving options
        static std::vector<const char*> global_map_coord_type_items = {"IMU", "LIDAR"};
        static std::vector<const char*> global_map_size_type_items = {"Tiled", "Single", "Submap"};

        ImGui::Checkbox("save global map enable", &save_global_map_);
        ImGui::BeginDisabled(!save_global_map_);
        ImGui::Combo("Global Map Coordinate Type", &global_map_coord_type_, global_map_coord_type_items.data(),
                     global_map_coord_type_items.size());
        ImGui::Combo("Global Map Size Type", &global_map_size_type_, global_map_size_type_items.data(),
                     global_map_size_type_items.size());
        if (global_map_size_type_ == static_cast<int>(GlobalMapSizeType::TILED)) {
            ImGui::SliderFloat("tile size", &global_map_tile_size_, 20.0f, 500.0f, "%.0f");
            global_map_tile_size_ = std::round(global_map_tile_size_ / 10.0f) * 10.0f;

            ImGui::Checkbox("downsample enable", &global_map_downsample_enable_);
            ImGui::SameLine();
            ImGui::BeginDisabled(!global_map_downsample_enable_);
            ImGui::SetNextItemWidth(160.0f);
            ImGui::DragFloat("voxel resolution", &global_map_voxel_resolution_, 0.01f, 0.01f, 5.0f, "%.2f");
            ImGui::EndDisabled();
        } else if (global_map_size_type_ == static_cast<int>(GlobalMapSizeType::SINGLE)) {
            ImGui::Checkbox("downsample enable", &global_map_downsample_enable_);
            ImGui::SameLine();
            ImGui::BeginDisabled(!global_map_downsample_enable_);
            ImGui::SetNextItemWidth(160.0f);
            ImGui::DragFloat("voxel resolution", &global_map_voxel_resolution_, 0.01f, 0.01f, 5.0f, "%.2f");
            ImGui::EndDisabled();
        }
        ImGui::EndDisabled();

        ImGui::Separator();
        // slam running result saving options
        ImGui::Checkbox("save SLAM running result enable", &save_slam_running_result_);

        ImGui::Separator();

        bool button_cancel = false;
        bool button_save = false;

        if (ImGui::Button("Save")) button_save = true;
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) button_cancel = true;

        if (button_save) {
            if (!save_output_path_.empty()) {
                slam_result_saver_->setInputFolder(save_input_path_);
                slam_result_saver_->setResultFolder(save_output_path_);

                if (save_traj_) {
                    slam_result_saver_->setTrajectoryEnable(static_cast<CoordType>(traj_coord_type_),
                                                            static_cast<TrajectoryFormat>(traj_format_),
                                                            static_cast<TrajectoryType>(traj_type_));
                }

                if (save_global_map_) {
                    slam_result_saver_->setGlobalMapEnable(static_cast<CoordType>(global_map_coord_type_),
                                                           static_cast<GlobalMapSizeType>(global_map_size_type_),
                                                           global_map_tile_size_, global_map_downsample_enable_,
                                                           global_map_voxel_resolution_);
                }

                if (save_slam_running_result_) { slam_result_saver_->setSLAMRunningResultEnable(); }

                progress_modal_->open<bool>("Saving SLAM Result", [this](guik::ProgressInterface& progress) {
                    return slam_result_saver_->run(progress);
                });
            } else {
                pfd::message("Invalid Output Path", "Please select a valid output folder before saving.",
                             pfd::choice::ok)
                    .result();
            }
        }

        std::optional<bool> save_result = progress_modal_->run<bool>("Saving SLAM Result");
        if (save_result.has_value()) {
            if (save_result.value()) {
                pfd::message("Save SLAM Result", "SLAM result saved successfully!", pfd::choice::ok).result();
                save_flag = true;
                this->resetSetting();
                ImGui::CloseCurrentPopup();
            } else {
                pfd::message("Save SLAM Result", "Failed to save SLAM result.", pfd::choice::ok).result();
            }
        }

        if (button_cancel) {
            this->resetSetting();
            ImGui::CloseCurrentPopup();
        }

        ImGui::PopStyleVar(2);
        ImGui::EndPopup();
    }

    return save_flag;
}

void SaveSLAMResultModal::resetSetting() {
    request_to_save_.store(false);
    save_input_path_.clear();
    save_output_path_.clear();

    slam_result_saver_.reset();
}

}  // namespace legkilo
