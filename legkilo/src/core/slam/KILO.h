#ifndef LEG_KILO_CORE_SLAM_KILO_H_
#define LEG_KILO_CORE_SLAM_KILO_H_

#include <deque>
#include <memory>
#include <string>

#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/Imu.h>

#include "common/eigen_types.hpp"
#include "common/pcl_types.h"
#include "common/sensor_types.hpp"
namespace legkilo {
class ESKF;
class StateInitial;
class VoxelMapManager;
}  // namespace legkilo

namespace legkilo {
class KILO {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit KILO(const std::string& config_file);
    ~KILO();

    bool process(common::MeasGroup measure, CloudPtr& cloud_down_body_out, CloudPtr& cloud_down_world_out,
                 size_t& success_pts_size_out);

    Vec3D getPos() const;
    Mat3D getRot() const; 

   private:
    // Initialization and helpers
    void initializeFromYaml(const std::string& config_file);
    void cloudLidarToWorld(const CloudPtr& cloud_lidar, CloudPtr& cloud_world);
    inline void pointLidarToWorld(const PointType& point_lidar, PointType& point_world);

    // Per-sensor handlers
    bool predictUpdateImu(const sensor_msgs::ImuPtr& imu);
    bool predictUpdateKinImu(const common::KinImuMeas& kin_imu);
    bool predictUpdatePoint(double current_time, size_t idx_i, size_t idx_j, const PointCloudType& cloud_down_body,
                            PointCloudType& cloud_down_world, size_t& success_pts_size_out);

   private:
    // Modules
    std::unique_ptr<ESKF> eskf_;
    std::unique_ptr<StateInitial> state_initial_;
    std::unique_ptr<VoxelMapManager> map_manager_;

    // Config/state
    bool imu_mode_only_ = true;  // true: IMU only; false: Kin+IMU
    double gravity_ = 9.81;
    double acc_norm_ = 1.0;
    double last_state_predict_time_ = 0.0;
    double last_state_update_time_ = 0.0;
    bool init_flag_ = true;

    // Extrinsics and downsampling
    Mat3D ext_rot_ = Mat3D::Identity();
    Vec3D ext_t_ = Vec3D::Zero();
    pcl::VoxelGrid<PointType> voxel_grid_;
};

}  // namespace legkilo

#endif  // LEG_KILO_CORE_SLAM_KILO_H_
