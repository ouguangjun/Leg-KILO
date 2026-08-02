#include "preprocess/lidar_processing.h"

namespace legkilo {
LidarProcessing::LidarProcessing(LidarProcessing::Config config) : config_(config) {
    LOG(INFO) << "Lidar Processing is Constructed";
    cloud_pcl_.reset(new PointCloudType());
    min_range2_ = config_.min_range_ * config_.min_range_;
    max_range2_ = config_.max_range_ * config_.max_range_;
}

LidarProcessing::~LidarProcessing() { LOG(INFO) << "Lidar Processing is Destructed"; }

common::LidarType LidarProcessing::getLidarType() const { return config_.lidar_type_; }

void LidarProcessing::processing(const ros_compat::PointCloud2MsgConstPtr& msg, common::LidarScan& lidar_scan) {
    switch (config_.lidar_type_) {
        case common::LidarType::VEL: velodyneHandler(msg, lidar_scan); break;

        case common::LidarType::OUSTER: ousterHander(msg, lidar_scan); break;

        case common::LidarType::HESAI: hesaiHandler(msg, lidar_scan); break;

        case common::LidarType::KITTI: kittiHandler(msg, lidar_scan); break;

        default: LOG(ERROR) << " Lidar Type is Not Currently Available"; break;
    }
}

void LidarProcessing::processing(const ros_compat::LivoxCustomMsgConstPtr& msg, common::LidarScan& lidar_scan) {
    livoxHandler(msg, lidar_scan);
}

void LidarProcessing::velodyneHandler(const ros_compat::PointCloud2MsgConstPtr& msg, common::LidarScan& lidar_scan) {
    lidar_scan.cloud_.reset(new PointCloudType());

    pcl::PointCloud<velodyne_ros::Point> cloud_pcl_raw;
    pcl::fromROSMsg(*msg, cloud_pcl_raw);

    auto [min_it, max_it] =
        std::minmax_element(cloud_pcl_raw.points.begin(), cloud_pcl_raw.points.end(),
                            [](const velodyne_ros::Point& a, const velodyne_ros::Point& b) { return a.time < b.time; });

    double first_point_time = config_.time_scale_ * min_it->time;
    double last_point_time = config_.time_scale_ * max_it->time;

    lidar_scan.lidar_begin_time_ = ros_compat::toSec(msg->header.stamp) + first_point_time;
    lidar_scan.lidar_end_time_ = ros_compat::toSec(msg->header.stamp) + last_point_time;

    int cloud_size = cloud_pcl_raw.points.size();
    lidar_scan.cloud_->points.reserve(cloud_size);

    for (int i = 0; i < cloud_size; ++i) {
        if ((i % config_.filter_num_) || rangeCheck(cloud_pcl_raw.points[i])) continue;
        if (nanCheck(cloud_pcl_raw.points[i])) continue;
        PointType added_point;
        added_point.x = cloud_pcl_raw.points[i].x;
        added_point.y = cloud_pcl_raw.points[i].y;
        added_point.z = cloud_pcl_raw.points[i].z;
        added_point.intensity = cloud_pcl_raw.points[i].intensity;
        double cur_point_time = config_.time_scale_ * cloud_pcl_raw.points[i].time;
        added_point.curvature = std::round((cur_point_time - first_point_time) * 500.0) / 500.0;

        lidar_scan.cloud_->points.push_back(added_point);
    }
}

void LidarProcessing::ousterHander(const ros_compat::PointCloud2MsgConstPtr& msg, common::LidarScan& lidar_scan) {
    lidar_scan.cloud_.reset(new PointCloudType());
    pcl::PointCloud<ouster_ros::Point> cloud_pcl_raw;
    pcl::fromROSMsg(*msg, cloud_pcl_raw);

    auto [min_it, max_it] =
        std::minmax_element(cloud_pcl_raw.points.begin(), cloud_pcl_raw.points.end(),
                            [](const ouster_ros::Point& a, const ouster_ros::Point& b) { return a.t < b.t; });

    double first_point_time = config_.time_scale_ * min_it->t;
    double last_point_time = config_.time_scale_ * max_it->t;

    lidar_scan.lidar_begin_time_ = ros_compat::toSec(msg->header.stamp) + first_point_time;
    lidar_scan.lidar_end_time_ = ros_compat::toSec(msg->header.stamp) + last_point_time;

    int cloud_size = cloud_pcl_raw.points.size();
    lidar_scan.cloud_->points.reserve(cloud_size);

    for (int i = 0; i < cloud_size; ++i) {
        if ((i % config_.filter_num_) || rangeCheck(cloud_pcl_raw.points[i])) continue;
        if (nanCheck(cloud_pcl_raw.points[i])) continue;
        PointType added_point;
        added_point.x = cloud_pcl_raw.points[i].x;
        added_point.y = cloud_pcl_raw.points[i].y;
        added_point.z = cloud_pcl_raw.points[i].z;
        added_point.intensity = cloud_pcl_raw.points[i].intensity;
        double cur_point_time = config_.time_scale_ * cloud_pcl_raw.points[i].t;
        added_point.curvature = std::round((cur_point_time - first_point_time) * 500.0) / 500.0;

        lidar_scan.cloud_->points.push_back(added_point);
    }
}

void LidarProcessing::hesaiHandler(const ros_compat::PointCloud2MsgConstPtr& msg, common::LidarScan& lidar_scan) {
    lidar_scan.cloud_.reset(new PointCloudType());
    pcl::PointCloud<hesai_ros::Point> cloud_pcl_raw;
    pcl::fromROSMsg(*msg, cloud_pcl_raw);

    auto [min_it, max_it] = std::minmax_element(
        cloud_pcl_raw.points.begin(), cloud_pcl_raw.points.end(),
        [](const hesai_ros::Point& a, const hesai_ros::Point& b) { return a.timestamp < b.timestamp; });

    double first_point_time = config_.time_scale_ * min_it->timestamp;
    double last_point_time = config_.time_scale_ * max_it->timestamp;

    lidar_scan.lidar_begin_time_ = first_point_time;
    lidar_scan.lidar_end_time_ = last_point_time;

    int cloud_size = cloud_pcl_raw.points.size();
    lidar_scan.cloud_->points.reserve(cloud_size);

    for (int i = 0; i < cloud_size; ++i) {
        if ((i % config_.filter_num_) || rangeCheck(cloud_pcl_raw.points[i])) continue;
        if (nanCheck(cloud_pcl_raw.points[i])) continue;
        PointType added_point;
        added_point.x = cloud_pcl_raw.points[i].x;
        added_point.y = cloud_pcl_raw.points[i].y;
        added_point.z = cloud_pcl_raw.points[i].z;
        added_point.intensity = cloud_pcl_raw.points[i].intensity;
        double cur_point_time = config_.time_scale_ * cloud_pcl_raw.points[i].timestamp;
        added_point.curvature = std::round((cur_point_time - first_point_time) * 500.0) / 500.0;

        lidar_scan.cloud_->points.push_back(added_point);
    }
}

void LidarProcessing::kittiHandler(const ros_compat::PointCloud2MsgConstPtr& msg, common::LidarScan& lidar_scan) {
    lidar_scan.cloud_.reset(new PointCloudType());

    pcl::PointCloud<kitti_ros::Point> cloud_pcl_raw;
    pcl::fromROSMsg(*msg, cloud_pcl_raw);

    static constexpr double kScanPeriod = 0.1;
    static constexpr double kVerticalAngleOffsetRad = 0.205 * M_PI / 180.0;
    static constexpr double kAxisNormEps = 1e-12;
    static const Eigen::Vector3d kZAxis(0.0, 0.0, 1.0);
    bool if_calib = true;

    const size_t cloud_size = cloud_pcl_raw.size();

    lidar_scan.lidar_begin_time_ = ros_compat::toSec(msg->header.stamp);
    lidar_scan.lidar_end_time_ = ros_compat::toSec(msg->header.stamp);

    lidar_scan.cloud_->points.reserve(cloud_size);

    for (size_t i = 0; i < cloud_size; ++i) {
        const auto& pt = cloud_pcl_raw.points[i];
        if ((i % config_.filter_num_) || rangeCheck(pt)) { continue; }
        if (nanCheck(pt)) { continue; }
        PointType q;
        q.x = pt.x;
        q.y = pt.y;
        q.z = pt.z;

        // KISS-ICP style KITTI scan correction:
        // rotation axis = p x z_axis, angle = 0.205 deg
        if (if_calib) {
            const Eigen::Vector3d p(static_cast<double>(q.x), static_cast<double>(q.y), static_cast<double>(q.z));
            const Eigen::Vector3d rotation_vec = p.cross(kZAxis);
            const double axis_norm = rotation_vec.norm();

            // Guard against degenerate normalization when point is near z-axis.
            if (axis_norm > kAxisNormEps) {
                const Eigen::Vector3d axis = rotation_vec / axis_norm;
                const Eigen::AngleAxisd aa(kVerticalAngleOffsetRad, axis);
                const Eigen::Vector3d p_corr = aa * p;
                q.x = static_cast<float>(p_corr.x());
                q.y = static_cast<float>(p_corr.y());
                q.z = static_cast<float>(p_corr.z());
            }
        }

        q.intensity = pt.intensity;
        q.curvature = 0.0f;

        lidar_scan.cloud_->points.push_back(q);
    }

    lidar_scan.cloud_->width = static_cast<uint32_t>(lidar_scan.cloud_->points.size());
    lidar_scan.cloud_->height = 1;
    lidar_scan.cloud_->is_dense = false;
}

void LidarProcessing::livoxHandler(const ros_compat::LivoxCustomMsgConstPtr& msg, common::LidarScan& lidar_scan) {
    lidar_scan.cloud_.reset(new PointCloudType());

    const auto& points = msg->points;
    const double timebase = ros_compat::toSec(msg->header.stamp);

    auto [min_it, max_it] = std::minmax_element(
        points.begin(), points.end(), [](const ros_compat::LivoxCustomPoint& a, const ros_compat::LivoxCustomPoint& b) {
            return a.offset_time < b.offset_time;
        });

    double first_point_time = config_.time_scale_ * min_it->offset_time;
    double last_point_time = config_.time_scale_ * max_it->offset_time;

    lidar_scan.lidar_begin_time_ = timebase + first_point_time;
    lidar_scan.lidar_end_time_ = timebase + last_point_time;

    const size_t cloud_size = points.size();
    lidar_scan.cloud_->points.reserve(cloud_size);

    for (size_t i = 1; i < cloud_size; ++i) {
        const auto& pt = points[i];
        if (!((pt.tag & 0x30) == 0x10) && !((pt.tag & 0x30) == 0x00)) continue;
        if ((i % config_.filter_num_) || rangeCheck(pt)) continue;
        if (nanCheck(pt)) continue;

        PointType added_point;
        added_point.x = pt.x;
        added_point.y = pt.y;
        added_point.z = pt.z;
        added_point.intensity = static_cast<float>(pt.reflectivity);
        double cur_point_time = config_.time_scale_ * pt.offset_time;
        added_point.curvature = std::round((cur_point_time - first_point_time) * 500.0) / 500.0;

        lidar_scan.cloud_->points.push_back(added_point);
    }

    lidar_scan.cloud_->width = static_cast<uint32_t>(lidar_scan.cloud_->points.size());
    lidar_scan.cloud_->height = 1;
    lidar_scan.cloud_->is_dense = false;
}

}  // namespace legkilo
