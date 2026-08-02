// This file is inspired by the design of https://github.com/koide3/glim (MIT License).
#include "viewer/viewer_slam_interface.h"

#include <glog/logging.h>
#include <algorithm>
#include <chrono>
#include <cmath>

#include "glk/colormap.hpp"
#include "glk/io/image_io.hpp"
#include "glk/pointcloud_buffer.hpp"
#include "glk/primitives/primitives.hpp"
#include "glk/texture.hpp"
#include "glk/thin_lines.hpp"
#include "guik/camera/orbit_camera_control_xy.hpp"
#include "guik/progress_modal.hpp"
#include "guik/viewer/light_viewer.hpp"
#include "portable-file-dialogs.h"

#include "common/yaml_helper.hpp"
#include "viewer/viewer_modal/save_slam_result_modal.h"

namespace {

class AutoOrbitCameraControlXY : public guik::OrbitCameraControlXY {
   public:
    using guik::OrbitCameraControlXY::OrbitCameraControlXY;

    void advanceYaw(double delta) { theta = std::remainder(theta + delta, 2.0 * M_PI); }
};

}  // namespace

namespace legkilo {

namespace {

using CloudColors = std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>>;

Eigen::Vector4f matchTypeColor(const LidarMatchType match_type) {
    switch (match_type) {
        case LidarMatchType::Point2Plane: return Eigen::Vector4f(1.0f, 0.5f, 0.0f, 1.0f);
        case LidarMatchType::Ndt: return Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f);
        case LidarMatchType::Unused:
        default: return Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f);
    }
}

CloudColors makeMatchColors(const ViewerSlamInterface::CloudVec& cloud, const LidarMatchTypesPtr& match_types) {
    CloudColors colors;
    colors.reserve(cloud.size());
    const bool has_match_types = match_types && match_types->size() == cloud.size();
    for (size_t i = 0; i < cloud.size(); ++i) {
        colors.emplace_back(matchTypeColor(has_match_types ? (*match_types)[i] : LidarMatchType::Unused));
    }
    return colors;
}

CloudColors makeHeightColors(const ViewerSlamInterface::CloudVec& cloud, const float alpha) {
    CloudColors colors;
    colors.reserve(cloud.size());
    if (cloud.empty()) return colors;

    static constexpr int kSkip = 10;
    static constexpr int kPctLoIdx = 5;
    static constexpr int kPctHiIdx = 95;

    std::vector<float> zs;
    zs.reserve(cloud.size() / kSkip + 1);
    for (size_t i = 0; i < cloud.size(); i += kSkip) { zs.push_back(cloud[i].z()); }

    std::sort(zs.begin(), zs.end());
    const size_t lo_idx = zs.size() * kPctLoIdx / 100;
    const size_t hi_idx = zs.size() * kPctHiIdx / 100;

    const float min_z = zs[lo_idx];
    const float max_z = zs[hi_idx];
    const float z_range = max_z - min_z;

    for (const auto& point : cloud) {
        const float ratio = z_range > 1e-3f ? (point.z() - min_z) / z_range : 0.5f;
        auto color = glk::colormapf(glk::COLORMAP::TURBO, ratio);
        color.w() = alpha;
        colors.emplace_back(color);
    }
    return colors;
}

}  // namespace

ViewerSlamInterface::ViewerSlamInterface(const std::string& config_path) {
    YamlHelper yaml_helper(config_path);
    request_slam_terminate_.store(false);

    draw_finished_submaps_ = true;
    draw_current_submap_ = true;
    draw_trajectory_ = true;
    draw_edges_ = true;
    follow_current_ = true;
    rotate_trajectory_ = false;
    max_submaps_display_ = -1;

    point_alpha_ = 0.4;
    point_size_ = 0.8;
    rotate_speed_deg_per_sec_ = 5.0f;
    result_save_path_ =
        std::string(ROOT_DIR) + "result/" + yaml_helper.get<std::string>("temp_result_save_folder", "temp") + "/";

    is_finished_drawable_update_ = false;
    is_current_keyframe_update_ = false;

    T_frontend_backend_.setIdentity();
    current_keyframe_.second = std::make_shared<CloudVec>();
}

ViewerSlamInterface::~ViewerSlamInterface() {
    this->stop();
    LOG(INFO) << "ViewerSlamInterface is destroyed";
}

void ViewerSlamInterface::workerLoop() {
    LOG(INFO) << "ViewerSlamInterface worker loop started";

    auto viewer = guik::LightViewer::instance(Eigen::Vector2i(2550, 1440));
    viewer->enable_vsync();
    viewer->show_info_window();
    viewer->set_camera_control(std::make_shared<AutoOrbitCameraControlXY>());

    int bg_width = 0;
    int bg_height = 0;
    std::vector<unsigned char> bg_bytes;
    const std::string bg_path = std::string(ROOT_DIR) + "config/viewer_background.png";
    if (glk::load_image(bg_path, bg_width, bg_height, bg_bytes)) {
        viewer->set_bg_texture(std::make_shared<glk::Texture>(Eigen::Vector2i(bg_width, bg_height), GL_RGBA, GL_RGBA,
                                                              GL_UNSIGNED_BYTE, bg_bytes.data()));
    } else {
        LOG(WARNING) << "Failed to load viewer background image: " << bg_path;
    }

    viewer->shader_setting().set_point_shape_circle();

    viewer->register_ui_callback("Menu", [this]() { this->menuCallback(); });
    viewer->register_ui_callback("Display Panel", [this]() { this->displayPanelCallback(); });
    viewer->register_ui_callback("Run Modal", [this]() { this->runModal(); });

    viewer->register_drawable_filter("filter", [this](const std::string& name) {
        const auto starts_with = [](const std::string& name, const std::string& pattern) {
            return name.size() < pattern.size() ? false : std::equal(pattern.begin(), pattern.end(), name.begin());
        };

        if (!draw_finished_submaps_ && starts_with(name, "finished_submap")) { return false; }

        if (!draw_current_submap_ && starts_with(name, "current_pointcloud")) { return false; }

        if (!draw_trajectory_ && starts_with(name, "current_traj")) { return false; }

        if (!draw_trajectory_ && starts_with(name, "finished_traj")) { return false; }

        if (!draw_edges_ && starts_with(name, "finished_edges")) { return false; }

        return true;
    });

    save_slam_result_modal_ = std::make_unique<SaveSLAMResultModal>();

    auto last_camera_update_time = std::chrono::steady_clock::now();
    while (!kill_switch_.load()) {
        const auto now = std::chrono::steady_clock::now();
        const double delta_seconds = std::chrono::duration<double>(now - last_camera_update_time).count();
        last_camera_update_time = now;
        this->updateCamera(delta_seconds);

        if (!viewer->spin_once()) {
            request_to_terminate_.store(true);
            kill_switch_.store(true);
            break;
        }

        std::lock_guard<std::mutex> lock(task_mutex_);
        for (const auto& func : task_deque_) { func(); }
        task_deque_.clear();
    }

    save_slam_result_modal_.reset();

    guik::LightViewer::destroy();
}

void ViewerSlamInterface::menuCallback() {
    bool request_to_quit = false;
    bool request_stop_slam = false;
    bool request_save_result = false;

    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("Close Viewer")) { request_to_quit = true; }

            if (ImGui::MenuItem("Stop SLAM")) { request_stop_slam = true; }

            if (ImGui::MenuItem("Save Result")) { request_save_result = true; }

            ImGui::EndMenu();
        }

        ImGui::EndMainMenuBar();
    }

    if (request_to_quit) {
        if (pfd::message("Close Viewer", "Are you sure you want to close the viewer?").result() == pfd::button::ok) {
            this->stop();
        }
    }

    if (request_stop_slam) {
        if (pfd::message("Stop SLAM", "Are you sure you want to stop the SLAM?").result() == pfd::button::ok) {
            this->requestSLAMTerminate();
        }
    }

    if (request_save_result) {
        if (pfd::message("Save Result", "Are you sure you want to save mapping result and stop the SLAM?").result() ==
            pfd::button::ok) {
            this->requestSLAMTerminate();
            save_slam_result_modal_->setInputPath(result_save_path_);
            save_slam_result_modal_->RequestToSave();
        }
    }
}

void ViewerSlamInterface::runModal() {
    bool save_flag = save_slam_result_modal_->run();
    if (save_flag) LOG(INFO) << "Save SLAM Result Success";
}

void ViewerSlamInterface::displayPanelCallback() {
    if (!ImGui::Begin("Selection", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) { 
        ImGui::End();   // 窗口折叠时也必须调用 End()
        return; 
    }

    const auto show_note = [](const std::string& text) {
        if (ImGui::IsItemHovered()) {
            ImGui::BeginTooltip();
            ImGui::Text("%s", text.c_str());
            ImGui::EndTooltip();
        }
        return false;
    };

    ImGui::Checkbox("Submap", &draw_finished_submaps_);
    show_note("Toggle the display of finished submaps.");

    ImGui::SameLine();
    ImGui::Checkbox("Current", &draw_current_submap_);
    show_note("Toggle the display of the current submap.");

    ImGui::Checkbox("Trajectory", &draw_trajectory_);
    show_note("Toggle the display of the trajectory.");

    ImGui::SameLine();
    ImGui::Checkbox("Edges", &draw_edges_);
    show_note("Toggle the display of the edges.");

    ImGui::Separator();
    ImGui::SetNextItemWidth(100.0f);
    ImGui::SliderInt("Max Submaps Display", &max_submaps_display_, -1, 500);
    show_note("Maximum number of submaps to display (-1 = all).");

    ImGui::Checkbox("Follow", &follow_current_);
    show_note("Camera follows the current pose.");

    if (ImGui::Checkbox("Rotate", &rotate_trajectory_) && rotate_trajectory_) {
        auto viewer = guik::LightViewer::instance();
        if (!std::dynamic_pointer_cast<AutoOrbitCameraControlXY>(viewer->get_camera_control())) {
            viewer->set_camera_control(std::make_shared<AutoOrbitCameraControlXY>());
        }
    }
    show_note("Camera orbits around the current target.");

    ImGui::SameLine();
    ImGui::SetNextItemWidth(140.0f);
    if (ImGui::DragFloat("Rotate speed", &rotate_speed_deg_per_sec_, 5.0f, 0.0f, 45.0f, "%.1f deg/s")) {
        rotate_speed_deg_per_sec_ = std::max(0.0f, rotate_speed_deg_per_sec_);
    }
    show_note("Camera orbit speed.");

    ImGui::End();
}

void ViewerSlamInterface::updateCamera(double delta_seconds) {
    if (!rotate_trajectory_) return;

    auto viewer = guik::LightViewer::instance();
    auto orbit_camera = std::dynamic_pointer_cast<AutoOrbitCameraControlXY>(viewer->get_camera_control());
    if (!orbit_camera) return;
    orbit_camera->advanceYaw(static_cast<double>(rotate_speed_deg_per_sec_) * M_PI / 180.0 * delta_seconds);
}

void ViewerSlamInterface::insertFinishedSubmap(NodeType id, SubmapPosesCloud submap_pose_cloud) {
    this->invoke([this, id, submap_pose_cloud]() {
        std::lock_guard<std::mutex> lock(mutex_full_);
        if (finished_submaps_.find(id) != finished_submaps_.end()) return;
        finished_submaps_.insert({id, submap_pose_cloud});
        this->updateTransformFrontend2Backend();

        // Cull old submap pointcloud drawables (keep only max_submaps_display_ most recent)
        if (max_submaps_display_ > 0) {
            const int num_submaps = static_cast<int>(finished_submaps_.size());
            const int visible_begin = std::max(0, num_submaps - max_submaps_display_);
            int counter = 0;
            auto viewer = guik::LightViewer::instance();
            for (auto& [sid, s_pose_cloud] : finished_submaps_) {
                if (counter < visible_begin) {
                    viewer->remove_drawable("finished_submap_" + std::to_string(sid));
                }
                ++counter;
            }
        }

        is_finished_drawable_update_ = true;
    });
}

void ViewerSlamInterface::insertCurrentKeyframe(NodeType id, const Pose& pose, CloudVecPtr keyframe_cloud,
                                                LidarMatchTypesPtr match_types) {
    this->invoke([this, id, pose, keyframe_cloud, match_types]() {
        std::lock_guard<std::mutex> lock(mutex_full_);
        if (current_submap_poses_.first != id) current_submap_poses_ = {id, {}};
        current_submap_poses_.second.emplace_back(pose);
        current_keyframe_ = {pose, keyframe_cloud};
        current_keyframe_match_types_ = match_types;
        is_current_keyframe_update_ = true;
    });
}

void ViewerSlamInterface::insertEdge(NodeType id1, NodeType id2) {
    this->invoke([this, id1, id2]() {
        std::lock_guard<std::mutex> lock(mutex_full_);
        if (finished_edges_.find(EdgePair(id1, id2)) != finished_edges_.end()) return;
        finished_edges_.insert(EdgePair(id1, id2));
        is_finished_drawable_update_ = true;
    });
}

void ViewerSlamInterface::updateFinishedSubmapPose(IDPosesPtr submap_poses) {
    this->invoke([this, submap_poses]() {
        std::lock_guard<std::mutex> lock(mutex_full_);
        for (const auto& [id, new_pose] : *submap_poses) {
            if (finished_submaps_.find(id) == finished_submaps_.end()) continue;
            std::get<0>(finished_submaps_[id]) = new_pose;
        }
        this->updateTransformFrontend2Backend();
        is_finished_drawable_update_ = true;
    });
}

void ViewerSlamInterface::updateTransformFrontend2Backend() {
    if (finished_submaps_.empty()) return;
    const auto& [last_submap_id, last_submap_pose_cloud] = *finished_submaps_.rbegin();
    const auto& [last_submap_pose, last_keyframe_poses, last_cloud] = last_submap_pose_cloud;
    T_frontend_backend_ = last_submap_pose * last_keyframe_poses.front().inverse();
}

void ViewerSlamInterface::refreshDrawables() {
    this->invoke([this]() {
        std::lock_guard<std::mutex> lock(mutex_full_);
        auto viewer = guik::LightViewer::instance();

        // finished
        if (is_finished_drawable_update_) {
            traj_finished_.clear();

            for (auto& [id, submap_pose_cloud] : finished_submaps_) {
                auto& [submap_pose, keyframe_poses, cloud] = submap_pose_cloud;

                // pointcloud drawable
                auto drawable = viewer->find_drawable("finished_submap_" + std::to_string(id));
                if (drawable.first) {
                    // Update transform for existing drawable
                    drawable.first->set_model_matrix(submap_pose);
                } else if (cloud && !cloud->empty()) {
                    auto pointcloud_drawable = std::make_shared<glk::PointCloudBuffer>(*cloud);
                    pointcloud_drawable->add_color(makeHeightColors(*cloud, point_alpha_));
                    auto shader_setting = guik::VertexColor(submap_pose.cast<float>())
                                              .set_alpha(point_alpha_)
                                              .set_point_scale(point_size_);
                    viewer->update_drawable("finished_submap_" + std::to_string(id), pointcloud_drawable,
                                            shader_setting);
                }
                // Release CPU buffer to save memory 
                if (cloud) cloud.reset();

                // origin drawable
                viewer->update_drawable(
                    "finished_coord_" + std::to_string(id), glk::Primitives::coordinate_system(),
                    guik::VertexColor(submap_pose.cast<float>() * Eigen::UniformScaling<float>(1.0f)));

                // traj
                for (const auto& keyframe_pose : keyframe_poses) {
                    Eigen::Isometry3d pose = submap_pose * keyframe_poses.front().inverse() * keyframe_pose;
                    traj_finished_.emplace_back(pose.translation().cast<float>());
                }
            }
            // traj drawable
            auto traj_finished_drawable = std::make_shared<glk::ThinLines>(traj_finished_, true);
            traj_finished_drawable->set_line_width(4.0f);
            viewer->update_drawable("finished_traj", traj_finished_drawable, guik::FlatWhite());

            // edge drawable
            std::vector<Eigen::Vector3f> edge_vec;
            for (const auto& edge : finished_edges_) {
                if (finished_submaps_.find(edge.first()) == finished_submaps_.end() ||
                    finished_submaps_.find(edge.second()) == finished_submaps_.end())
                    continue;
                edge_vec.emplace_back(std::get<0>(finished_submaps_[edge.first()]).translation().cast<float>());
                edge_vec.emplace_back(std::get<0>(finished_submaps_[edge.second()]).translation().cast<float>());
            }
            auto edge_drawable = std::make_shared<glk::ThinLines>(edge_vec, false);
            edge_drawable->set_line_width(4.0f);
            viewer->update_drawable("finished_edges", edge_drawable, guik::FlatRed());

            is_finished_drawable_update_ = false;
        }

        // current
        if (is_current_keyframe_update_ && current_keyframe_.second && !current_keyframe_.second->empty()) {
            std::vector<Eigen::Vector3f> traj_current;
            for (const auto& keyframe_pose : current_submap_poses_.second) {
                auto update_pose = T_frontend_backend_ * keyframe_pose;
                traj_current.emplace_back(update_pose.translation().cast<float>());
            }

            auto traj_current_drawable = std::make_shared<glk::ThinLines>(traj_current, true);
            traj_current_drawable->set_line_width(5.0f);
            viewer->update_drawable("current_traj", traj_current_drawable, guik::FlatBlue());

            Pose current_pose = T_frontend_backend_ * current_keyframe_.first;
            viewer->update_drawable("current_coord", glk::Primitives::coordinate_system(),
                                    guik::VertexColor(current_pose.cast<float>() * Eigen::UniformScaling<float>(1.0f)));

            auto pointcloud_drawable = std::make_shared<glk::PointCloudBuffer>(*current_keyframe_.second);
            auto colors = makeMatchColors(*current_keyframe_.second, current_keyframe_match_types_);
            pointcloud_drawable->add_color(colors);
            auto shader_setting = guik::VertexColor(current_pose.cast<float>()).set_alpha(1.0).set_point_scale(2.0f);

            viewer->update_drawable("current_pointcloud", pointcloud_drawable, shader_setting);

            if (follow_current_ || rotate_trajectory_) { viewer->lookat(current_pose.translation().cast<float>()); }

            is_current_keyframe_update_ = false;
        }
    });
}

}  // namespace legkilo
