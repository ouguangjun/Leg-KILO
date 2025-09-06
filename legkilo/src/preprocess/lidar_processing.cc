#include "preprocess/lidar_processing.h"

namespace legkilo {
LidarProcessing::LidarProcessing(LidarProcessing::Config config) : config_(config) {
    LOG(INFO) << "Lidar Processing is Constructed";
    cloud_pcl_.reset(new PointCloudType());
}

LidarProcessing::~LidarProcessing() { LOG(INFO) << "Lidar Processing is Destructed"; }

common::LidarType LidarProcessing::getLidarType() const { return config_.lidar_type_; }

void LidarProcessing::processing(const sensor_msgs::PointCloud2::ConstPtr& msg, common::LidarScan& lidar_scan) {
    switch (config_.lidar_type_) {
        case common::LidarType::VEL: velodyneHandler(msg, lidar_scan); break;

        case common::LidarType::OUSTER: ousterHander(msg, lidar_scan); break;

        default: LOG(ERROR) << " Lidar Type is Not Currently Available"; break;
    }
}

void LidarProcessing::velodyneHandler(const sensor_msgs::PointCloud2::ConstPtr& msg, common::LidarScan& lidar_scan) {
    lidar_scan.cloud_.reset(new PointCloudType());

    pcl::PointCloud<velodyne_ros::Point> cloud_pcl_raw;
    pcl::fromROSMsg(*msg, cloud_pcl_raw);

    float first_point_time = config_.time_scale_ * cloud_pcl_raw.points.front().time;
    float last_point_time = config_.time_scale_ * cloud_pcl_raw.points.back().time;

    lidar_scan.lidar_begin_time_ = msg->header.stamp.toSec() + first_point_time;
    lidar_scan.lidar_end_time_ = msg->header.stamp.toSec() + last_point_time;

    int cloud_size = cloud_pcl_raw.points.size();
    lidar_scan.cloud_->points.reserve(cloud_size);

    for (int i = 0; i < cloud_size; ++i) {
        if ((i % config_.filter_num_) || blindCheck(cloud_pcl_raw.points[i])) continue;
        PointType added_point;
        added_point.x = cloud_pcl_raw.points[i].x;
        added_point.y = cloud_pcl_raw.points[i].y;
        added_point.z = cloud_pcl_raw.points[i].z;
        added_point.intensity = cloud_pcl_raw.points[i].intensity;
        float cur_point_time = config_.time_scale_ * cloud_pcl_raw.points[i].time;
        added_point.curvature = std::round((cur_point_time - first_point_time) * 500.0f) / 500.0f;

        lidar_scan.cloud_->points.push_back(added_point);
    }
}

void LidarProcessing::ousterHander(const sensor_msgs::PointCloud2::ConstPtr& msg, common::LidarScan& lidar_scan) {
    lidar_scan.cloud_.reset(new PointCloudType());
    pcl::PointCloud<ouster_ros::Point> cloud_pcl_raw;
    pcl::fromROSMsg(*msg, cloud_pcl_raw);

    float first_point_time = config_.time_scale_ * cloud_pcl_raw.points.front().t;
    float last_point_time = config_.time_scale_ * cloud_pcl_raw.points.back().t;

    lidar_scan.lidar_begin_time_ = msg->header.stamp.toSec() + first_point_time;
    lidar_scan.lidar_end_time_ = msg->header.stamp.toSec() + last_point_time;

    int cloud_size = cloud_pcl_raw.points.size();
    lidar_scan.cloud_->points.reserve(cloud_size);

    for (int i = 0; i < cloud_size; ++i) {
        if ((i % config_.filter_num_) || blindCheck(cloud_pcl_raw.points[i])) continue;
        PointType added_point;
        added_point.x = cloud_pcl_raw.points[i].x;
        added_point.y = cloud_pcl_raw.points[i].y;
        added_point.z = cloud_pcl_raw.points[i].z;
        added_point.intensity = cloud_pcl_raw.points[i].intensity;
        float cur_point_time = config_.time_scale_ * cloud_pcl_raw.points[i].t;
        added_point.curvature = std::round((cur_point_time - first_point_time) * 500.0f) / 500.0f;

        lidar_scan.cloud_->points.push_back(added_point);
    }
}

}  // namespace legkilo
