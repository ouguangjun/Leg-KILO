#include "interface/ros1/ros_interface.h"

namespace legkilo {

const bool time_list(PointType& x, PointType& y) { return (x.curvature < y.curvature); }

#define THREAD_SLEEP(ms) std::this_thread::sleep_for(std::chrono::milliseconds(ms))

RosInterface::RosInterface(ros::NodeHandle& nh) : nh_(nh) {
    LOG(INFO) << "Ros Interface is Constructed";
    pub_odom_world_ = nh_.advertise<nav_msgs::Odometry>("/Odomtry", 10000);
    pub_path_ = nh.advertise<nav_msgs::Path>("/path", 10000);
    pub_pointcloud_world_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 10000);
    pub_pointcloud_body_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 10000);
    if (pub_joint_tf_enable_) { pub_joint_state_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10000); }

    odom_world_.header.frame_id = "camera_init";
    odom_world_.child_frame_id = "base";
    path_world_.header.frame_id = "camera_init";
    path_world_.header.stamp = ros::Time::now();
    pose_path_.header.frame_id = "camera_init";
}

RosInterface::~RosInterface() {
    LOG(INFO) << "Ros Interface is Destructed";
    if (lidar_thread_ && lidar_thread_->joinable()) { lidar_thread_->join(); }
    if (imu_thread_ && imu_thread_->joinable()) { imu_thread_->join(); }
    if (kinematic_thread_ && kinematic_thread_->joinable()) { kinematic_thread_->join(); }
}

bool RosInterface::initParamAndReset(const std::string& config_file) {
    YamlHelper yaml_helper(config_file);

    /* Topic and options*/
    options::kLidarTopic = yaml_helper.get<std::string>("lidar_topic");
    options::kImuUse = yaml_helper.get<bool>("only_imu_use", true);
    options::kKinAndImuUse = static_cast<bool>(!options::kImuUse);
    options::kRedundancy = yaml_helper.get<bool>("redundancy", false);
    if (options::kImuUse) { options::kImuTopic = yaml_helper.get<std::string>("imu_topic"); }
    if (options::kKinAndImuUse) { options::kKinematicTopic = yaml_helper.get<std::string>("kinematic_topic"); }

    /* ESKF */
    ESKF::Config eskf_config;
    eskf_config.vel_process_cov = yaml_helper.get<double>("vel_process_cov");
    eskf_config.imu_acc_process_cov = yaml_helper.get<double>("imu_acc_process_cov");
    eskf_config.imu_gyr_process_cov = yaml_helper.get<double>("imu_gyr_process_cov");
    eskf_config.acc_bias_process_cov = yaml_helper.get<double>("acc_bias_process_cov");
    eskf_config.gyr_bias_process_cov = yaml_helper.get<double>("gyr_bias_process_cov");
    eskf_config.kin_bias_process_cov = yaml_helper.get<double>("kin_bias_process_cov");
    eskf_config.contact_process_cov = yaml_helper.get<double>("contact_process_cov");

    eskf_config.imu_acc_meas_noise = yaml_helper.get<double>("imu_acc_meas_noise");
    eskf_config.imu_acc_z_meas_noise = yaml_helper.get<double>("imu_acc_z_meas_noise");
    eskf_config.imu_gyr_meas_noise = yaml_helper.get<double>("imu_gyr_meas_noise");
    eskf_config.kin_meas_noise = yaml_helper.get<double>("kin_meas_noise");
    eskf_config.chd_meas_noise = yaml_helper.get<double>("chd_meas_noise");
    eskf_config.contact_meas_noise = yaml_helper.get<double>("contact_meas_noise");
    eskf_config.lidar_point_meas_ratio = yaml_helper.get<double>("lidar_point_meas_ratio");
    eskf_ = std::make_unique<ESKF>(eskf_config);

    /* kin. and imu processing*/
    init_time_ = yaml_helper.get<double>("init_time", (double)0.1);
    gravity_ = yaml_helper.get<double>("gravity", (double)9.81);
    if (options::kImuUse) {
        state_initial_ = std::make_unique<StateInitialByImu>(gravity_);
    } else {
        state_initial_ = std::make_unique<StateInitialByKinImu>(gravity_);
    }

    satu_acc_ = yaml_helper.get<double>("satu_acc", 35);
    satu_gyr_ = yaml_helper.get<double>("satu_gyr", 30);

    /* kinematics*/
    Kinematics::Config kinematics_config;
    kinematics_config.leg_offset_x = yaml_helper.get<double>("leg_offset_x");
    kinematics_config.leg_offset_y = yaml_helper.get<double>("leg_offset_y");
    kinematics_config.leg_calf_length = yaml_helper.get<double>("leg_calf_length");
    kinematics_config.leg_thigh_length = yaml_helper.get<double>("leg_thigh_length");
    kinematics_config.leg_thigh_offset = yaml_helper.get<double>("leg_thigh_offset");
    kinematics_config.contact_force_threshold_up = yaml_helper.get<double>("contact_force_threshold_up");
    kinematics_config.contact_force_threshold_down = yaml_helper.get<double>("contact_force_threshold_down");
    kinematics_ = std::make_unique<Kinematics>(kinematics_config);

    /* lidar processing*/
    LidarProcessing::Config lidar_process_config;
    lidar_process_config.blind_ = yaml_helper.get<float>("blind");
    lidar_process_config.filter_num_ = yaml_helper.get<int>("filter_num");
    lidar_process_config.time_scale_ = yaml_helper.get<double>("time_scale");
    lidar_process_config.point_stamp_correct_ = yaml_helper.get<bool>("point_stamp_correct", false);
    lidar_process_config.lidar_type_ = static_cast<common::LidarType>(yaml_helper.get<int>("lidar_type"));
    lidar_processing_ = std::make_unique<LidarProcessing>(lidar_process_config);

    /* voxel grid */
    float voxel_grid_resolution = yaml_helper.get<float>("voxel_grid_resolution");
    // voxel_grid_ = std::make_unique<VoxelGrid>(voxel_grid_resolution);
    voxel_grid_.setLeafSize(voxel_grid_resolution, voxel_grid_resolution, voxel_grid_resolution);

    /* voxel map*/
    VoxelMapConfig voxel_map_config;
    voxel_map_config.is_pub_plane_map_ = yaml_helper.get<bool>("pub_plane_en");
    voxel_map_config.max_layer_ = yaml_helper.get<int>("max_layer");
    voxel_map_config.max_voxel_size_ = yaml_helper.get<double>("voxel_size");
    voxel_map_config.planner_threshold_ = yaml_helper.get<double>("min_eigen_value");
    voxel_map_config.sigma_num_ = yaml_helper.get<double>("sigma_num");
    voxel_map_config.beam_err_ = yaml_helper.get<double>("beam_err");
    voxel_map_config.dept_err_ = yaml_helper.get<double>("dept_err");
    voxel_map_config.layer_init_num_ = yaml_helper.get<std::vector<int>>("layer_init_num");
    voxel_map_config.max_points_num_ = yaml_helper.get<int>("max_points_num");
    voxel_map_config.map_sliding_en = yaml_helper.get<bool>("map_sliding_en");
    voxel_map_config.half_map_size = yaml_helper.get<int>("half_map_size");
    voxel_map_config.sliding_thresh = yaml_helper.get<double>("sliding_thresh");
    map_manager_ = std::make_unique<VoxelMapManager>(voxel_map_config);

    /* Extrinsic*/
    std::vector<double> ext_t = yaml_helper.get<std::vector<double>>("extrinsic_T");
    std::vector<double> ext_R = yaml_helper.get<std::vector<double>>("extrinsic_R");
    ext_rot_ << MAT_FROM_ARRAY(ext_R);
    ext_t_ << VEC_FROM_ARRAY(ext_t);
    map_manager_->extT_ = ext_t_;
    map_manager_->extR_ = ext_rot_;

    /* Visualizaition*/
    pub_joint_tf_enable_ = yaml_helper.get<bool>("pub_joint_tf_enable");

    return true;
}

void RosInterface::rosInit(const std::string& config_file) {
    this->initParamAndReset(config_file);
    this->subscribeLidar();

    if (options::kImuUse) { this->subscribeImu(); }

    if (options::kKinAndImuUse) { this->subscribeKinematicImu(); }
}

void RosInterface::subscribeLidar() {
    this->lidar_thread_ = std::unique_ptr<std::thread>(new std::thread(&RosInterface::lidarLoop, this));
    THREAD_SLEEP(100);
}

void RosInterface::subscribeImu() {
    this->imu_thread_ = std::unique_ptr<std::thread>(new std::thread(&RosInterface::imuLoop, this));
    THREAD_SLEEP(100);
}

void RosInterface::subscribeKinematicImu() {
    this->kinematic_thread_ = std::unique_ptr<std::thread>(new std::thread(&RosInterface::kinematicImuLoop, this));
    THREAD_SLEEP(100);
}

void RosInterface::lidarLoop() {
    LOG(INFO) << "Lidar Loop Begin";

    ros::NodeHandle nh(nh_, "lidar_sub");
    ros::CallbackQueue queue;
    nh.setCallbackQueue(&queue);
    this->sub_lidar_raw_ =
        nh.subscribe<sensor_msgs::PointCloud2>(options::kLidarTopic, 1000, &RosInterface::lidarCallBack, this);

    while (ros::ok() && !options::FLAG_EXIT.load()) { queue.callAvailable(ros::WallDuration(0.2)); }
}

void RosInterface::imuLoop() {
    LOG(INFO) << "IMU Loop Begin";

    ros::NodeHandle nh(nh_, "imu_sub");
    ros::CallbackQueue queue;
    nh.setCallbackQueue(&queue);
    this->sub_imu_raw_ = nh.subscribe(options::kImuTopic, 10000, &RosInterface::imuCallBack, this);

    while (ros::ok() && !options::FLAG_EXIT.load()) { queue.callAvailable(ros::WallDuration(0.1)); }
}

void RosInterface::kinematicImuLoop() {
    LOG(INFO) << "Kinematic Loop Begin";

    ros::NodeHandle nh(nh_, "kinematic_sub");
    ros::CallbackQueue queue;
    nh.setCallbackQueue(&queue);
    this->sub_kinematic_raw_ = nh.subscribe(options::kKinematicTopic, 10000, &RosInterface::kinematicImuCallBack, this);

    while (ros::ok() && !options::FLAG_EXIT.load()) { queue.callAvailable(ros::WallDuration(0.1)); }
}

void RosInterface::lidarCallBack(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    mutex_.lock();
    static double last_scan_time = msg->header.stamp.toSec();

    Timer::measure("Lidar Processing", [&, this]() {
        if (msg->header.stamp.toSec() < last_scan_time) {
            LOG(WARNING) << "Time inconsistency detected in Lidar data stream";
            lidar_cache_.clear();
        }

        common::LidarScan lidar_scan;
        lidar_processing_->processing(msg, lidar_scan);
        lidar_cache_.push_back(lidar_scan);
        last_scan_time = msg->header.stamp.toSec();
    });

    last_scan_time = msg->header.stamp.toSec();
    mutex_.unlock();
    return;
}

void RosInterface::imuCallBack(const sensor_msgs::Imu::ConstPtr& msg) {
    static sensor_msgs::Imu last_imu_msg;
    sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu(*msg));

    if (options::kRedundancy) {
        if (imu_msg->linear_acceleration.z == last_imu_msg.linear_acceleration.z &&
            imu_msg->angular_velocity.z == last_imu_msg.angular_velocity.z) {
            last_imu_msg = *imu_msg;
            return;
        }
    }

    double timestamp = imu_msg->header.stamp.toSec();
    mutex_.lock();
    if (timestamp < last_timestamp_imu_) {
        LOG(WARNING) << "Time inconsistency detected in Imu data stream";
        imu_cache_.clear();
    }

    imu_cache_.push_back(imu_msg);
    last_imu_msg = *imu_msg;
    last_timestamp_imu_ = timestamp;
    mutex_.unlock();
    return;
}

void RosInterface::kinematicImuCallBack(const unitree_legged_msgs::HighState::ConstPtr& msg) {
    static unitree_legged_msgs::HighState last_highstate_msg;
    unitree_legged_msgs::HighStatePtr highstate_msg(new unitree_legged_msgs::HighState(*msg));

    if (options::kRedundancy) {
        if (highstate_msg->imu.accelerometer[2] == last_highstate_msg.imu.accelerometer[2] &&
            highstate_msg->imu.gyroscope[2] == last_highstate_msg.imu.gyroscope[2]) {
            last_highstate_msg = *highstate_msg;
            return;
        }
    }

    double timestamp = highstate_msg->stamp.toSec();
    mutex_.lock();
    if (timestamp < last_timestamp_kin_imu_) {
        LOG(WARNING) << "Time inconsistency detected in Kin. Imu data stream";
        kin_imu_cache_.clear();
    }

    common::KinImuMeas kin_imu_meas;

    kinematics_->processing(*highstate_msg, kin_imu_meas);

    kin_imu_cache_.push_back(kin_imu_meas);
    last_timestamp_kin_imu_ = timestamp;
    last_highstate_msg = *highstate_msg;
    mutex_.unlock();

    if (pub_joint_tf_enable_) {
        static std::vector<std::string> joint_names = {
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = last_highstate_msg.stamp;
        joint_state.name = joint_names;
        const auto& motor = last_highstate_msg.motorState;
        joint_state.position = {
            motor[0].q, motor[1].q, motor[2].q, motor[3].q, motor[4].q,  motor[5].q,
            motor[6].q, motor[7].q, motor[8].q, motor[9].q, motor[10].q, motor[11].q,
        };
        joint_state.velocity = {
            motor[0].dq, motor[1].dq, motor[2].dq, motor[3].dq, motor[4].dq,  motor[5].dq,
            motor[6].dq, motor[7].dq, motor[8].dq, motor[9].dq, motor[10].dq, motor[11].dq,
        };
        pub_joint_state_.publish(joint_state);
    }
    return;
}

bool RosInterface::syncPackage() {
    static bool lidar_push_ = false;

    std::lock_guard<std::mutex> lk(mutex_);

    // pack lidar and  imu
    if (options::kImuUse) {
        if (lidar_cache_.empty() || imu_cache_.empty()) return false;

        if (!lidar_push_) {
            measure_.lidar_scan_ = lidar_cache_.front();
            lidar_end_time_ = measure_.lidar_scan_.lidar_end_time_;
            lidar_push_ = true;
        }

        if (last_timestamp_imu_ < lidar_end_time_) { return false; }

        double imu_time = imu_cache_.front()->header.stamp.toSec();
        measure_.imus_.clear();
        while ((!imu_cache_.empty()) && (imu_time < lidar_end_time_)) {
            imu_time = imu_cache_.front()->header.stamp.toSec();
            if (imu_time > lidar_end_time_) break;
            measure_.imus_.push_back(imu_cache_.front());
            imu_cache_.pop_front();
        }

        lidar_cache_.pop_front();
        lidar_push_ = false;

        return true;
    }

    // pack lidar, kin. and  imu
    if (options::kKinAndImuUse) {
        if (lidar_cache_.empty() || kin_imu_cache_.empty()) return false;

        if (!lidar_push_) {
            measure_.lidar_scan_ = lidar_cache_.front();
            lidar_end_time_ = measure_.lidar_scan_.lidar_end_time_;
            lidar_push_ = true;
        }

        if (last_timestamp_kin_imu_ < lidar_end_time_) { return false; }

        double kin_imu_time = kin_imu_cache_.front().time_stamp_;
        measure_.kin_imus_.clear();
        while ((!kin_imu_cache_.empty()) && (kin_imu_time < lidar_end_time_)) {
            kin_imu_time = kin_imu_cache_.front().time_stamp_;
            if (kin_imu_time > lidar_end_time_) break;
            measure_.kin_imus_.push_back(kin_imu_cache_.front());
            kin_imu_cache_.pop_front();
        }

        lidar_cache_.pop_front();
        lidar_push_ = false;

        return true;
    }

    throw std::runtime_error("Error sync package");
    return false;
}

inline void RosInterface::pointLidarToImu(const PointType& point_lidar, PointType& point_imu) {
    Eigen::Vector3d pt_lidar(point_lidar.x, point_lidar.y, point_lidar.z);
    Eigen::Vector3d pt_imu = ext_rot_ * pt_lidar + ext_t_;

    point_imu.x = static_cast<float>(pt_imu(0));
    point_imu.y = static_cast<float>(pt_imu(1));
    point_imu.z = static_cast<float>(pt_imu(2));
    point_imu.intensity = point_lidar.intensity;
}

inline void RosInterface::pointLidarToWorld(const PointType& point_lidar, PointType& point_world) {
    Eigen::Vector3d pt_lidar(point_lidar.x, point_lidar.y, point_lidar.z);
    Eigen::Vector3d pt_imu = ext_rot_ * pt_lidar + ext_t_;
    Eigen::Vector3d pt_world = eskf_->state().rot_ * pt_imu + eskf_->state().pos_;

    point_world.x = static_cast<float>(pt_world(0));
    point_world.y = static_cast<float>(pt_world(1));
    point_world.z = static_cast<float>(pt_world(2));
    point_world.intensity = point_lidar.intensity;
}

void RosInterface::cloudLidarToWorld(const CloudPtr& cloud_lidar, CloudPtr& cloud_world) {
    cloud_world->clear();
    cloud_world->points.resize(cloud_lidar->points.size());
    for (size_t i = 0; i < cloud_lidar->points.size(); ++i) {
        pointLidarToWorld(cloud_lidar->points[i], cloud_world->points[i]);
    }
}

void RosInterface::publishOdomTFPath(double end_time) {
    // odometry
    odom_world_.header.stamp = ros::Time().fromSec(end_time);
    odom_world_.pose.pose.position.x = eskf_->state().pos_(0);
    odom_world_.pose.pose.position.y = eskf_->state().pos_(1);
    odom_world_.pose.pose.position.z = eskf_->state().pos_(2);
    q_eigen_ = Eigen::Quaterniond(eskf_->getRot());
    odom_world_.pose.pose.orientation.w = q_eigen_.w();
    odom_world_.pose.pose.orientation.x = q_eigen_.x();
    odom_world_.pose.pose.orientation.y = q_eigen_.y();
    odom_world_.pose.pose.orientation.z = q_eigen_.z();
    pub_odom_world_.publish(odom_world_);

    // tf
    transform_.setOrigin(tf::Vector3(odom_world_.pose.pose.position.x, odom_world_.pose.pose.position.y,
                                     odom_world_.pose.pose.position.z));
    q_tf_.setW(odom_world_.pose.pose.orientation.w);
    q_tf_.setX(odom_world_.pose.pose.orientation.x);
    q_tf_.setY(odom_world_.pose.pose.orientation.y);
    q_tf_.setZ(odom_world_.pose.pose.orientation.z);
    transform_.setRotation(q_tf_);
    br_.sendTransform(tf::StampedTransform(transform_, odom_world_.header.stamp, "camera_init", "base"));

    // path
    pose_path_.header.stamp = odom_world_.header.stamp;
    pose_path_.pose = odom_world_.pose.pose;
    path_world_.poses.push_back(pose_path_);
    pub_path_.publish(path_world_);
}

void RosInterface::publishPointcloudWorld(double end_time) {
    sensor_msgs::PointCloud2 pcl_msg;
    pcl::toROSMsg(*cloud_down_world_, pcl_msg);
    pcl_msg.header.stamp = ros::Time().fromSec(end_time);
    pcl_msg.header.frame_id = "camera_init";
    pub_pointcloud_world_.publish(pcl_msg);
}

bool RosInterface::predictUpdatePoint(double current_time, size_t idx_i, size_t idx_j) {
    /* 1. predict state*/
    double dt_cov = current_time - last_state_update_time_;
    eskf_->predict(dt_cov, false, true);
    double dt = current_time - last_state_predict_time_;
    eskf_->predict(dt, true, false);
    last_state_predict_time_ = current_time;

    /* 2. residual compute*/
    size_t points_size = idx_j - idx_i;
    std::vector<PointToPlane> ptpl_list;
    std::vector<pointWithVar> pv_list(points_size);
    ptpl_list.reserve(points_size);
    for (size_t i = 0; i < points_size; ++i) {
        PointType& cur_pt = cloud_down_body_->points[i + idx_i];

        /* 2.1 point var(body and world) compute*/
        pointWithVar& cur_pt_var = pv_list[i];
        cur_pt_var.point_b << cur_pt.x, cur_pt.y, cur_pt.z;
        cur_pt_var.point_i = ext_rot_ * cur_pt_var.point_b + ext_t_;
        cur_pt_var.point_w = eskf_->state().rot_ * cur_pt_var.point_i + eskf_->state().pos_;
        cloud_down_world_->points[idx_i + i].x = cur_pt_var.point_w(0);
        cloud_down_world_->points[idx_i + i].y = cur_pt_var.point_w(1);
        cloud_down_world_->points[idx_i + i].z = cur_pt_var.point_w(2);
        cloud_down_world_->points[idx_i + i].intensity = 0;
        calcBodyCov(cur_pt_var.point_b, map_manager_->config_setting_.dept_err_,
                    map_manager_->config_setting_.beam_err_, cur_pt_var.body_var);
        cur_pt_var.point_crossmat << SKEW_SYM_MATRIX(cur_pt_var.point_i);
        M3D rot_extR = eskf_->state().rot_ * ext_rot_;
        M3D rot_crossmat = eskf_->state().rot_ * cur_pt_var.point_crossmat;
        cur_pt_var.var = rot_extR * cur_pt_var.body_var * rot_extR.transpose() +
                         rot_crossmat * eskf_->cov().block<3, 3>(0, 0) * rot_crossmat.transpose() +
                         eskf_->cov().block<3, 3>(3, 3);

        /* 2.2 point residual */
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = cur_pt_var.point_w[j] / map_manager_->config_setting_.max_voxel_size_;
            if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
        }
        VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
        auto iter = map_manager_->voxel_map_.find(position);
        if (iter != map_manager_->voxel_map_.end()) {
            VoxelOctoTree* current_octo = iter->second;
            PointToPlane single_ptpl;
            bool is_success = false;
            double prob = 0;
            map_manager_->build_single_residual(cur_pt_var, current_octo, 0, is_success, prob, single_ptpl);
            if (!is_success) {
                VOXEL_LOCATION near_position = position;
                if (loc_xyz[0] > (current_octo->voxel_center_[0] + current_octo->quater_length_)) {
                    near_position.x = near_position.x + 1;
                } else if (loc_xyz[0] < (current_octo->voxel_center_[0] - current_octo->quater_length_)) {
                    near_position.x = near_position.x - 1;
                }
                if (loc_xyz[1] > (current_octo->voxel_center_[1] + current_octo->quater_length_)) {
                    near_position.y = near_position.y + 1;
                } else if (loc_xyz[1] < (current_octo->voxel_center_[1] - current_octo->quater_length_)) {
                    near_position.y = near_position.y - 1;
                }
                if (loc_xyz[2] > (current_octo->voxel_center_[2] + current_octo->quater_length_)) {
                    near_position.z = near_position.z + 1;
                } else if (loc_xyz[2] < (current_octo->voxel_center_[2] - current_octo->quater_length_)) {
                    near_position.z = near_position.z - 1;
                }
                auto iter_near = map_manager_->voxel_map_.find(near_position);
                if (iter_near != map_manager_->voxel_map_.end()) {
                    map_manager_->build_single_residual(cur_pt_var, iter_near->second, 0, is_success, prob,
                                                        single_ptpl);
                }
            }
            if (is_success) {
                ++success_pts_size;
                ptpl_list.push_back(single_ptpl);
            }
        }
    }

    /* 3. kalman filter update*/
    size_t effect_num = ptpl_list.size();
    bool eskf_update = effect_num > 0;
    if (eskf_update) {
        ObsShared obs_shared;
        obs_shared.pt_h.resize(effect_num, 6);
        obs_shared.pt_R.resize(effect_num);
        obs_shared.pt_z.resize(effect_num);
        for (size_t k = 0; k < effect_num; ++k) {
            V3D crossmat_rotT_u = ptpl_list[k].point_crossmat_ * eskf_->state().rot_.transpose() * ptpl_list[k].normal_;
            obs_shared.pt_h.row(k) << crossmat_rotT_u(0), crossmat_rotT_u(1), crossmat_rotT_u(2),
                ptpl_list[k].normal_(0), ptpl_list[k].normal_(1), ptpl_list[k].normal_(2);

            obs_shared.pt_z(k) = -ptpl_list[k].dis_to_plane_;

            Eigen::Matrix<double, 1, 6> J_nq;
            J_nq.block<1, 3>(0, 0) = ptpl_list[k].point_w_ - ptpl_list[k].center_;
            J_nq.block<1, 3>(0, 3) = -ptpl_list[k].normal_;
            M3D var;
            var = eskf_->state().rot_ * ext_rot_ * ptpl_list[k].body_cov_ * ext_rot_.transpose() *
                  eskf_->state().rot_.transpose();
            double single_l = J_nq * ptpl_list[k].plane_var_ * J_nq.transpose();
            obs_shared.pt_R(k) = eskf_->config().lidar_point_meas_ratio *
                                 (single_l + ptpl_list[k].normal_.transpose() * var * ptpl_list[k].normal_);
        }
        eskf_->updateByPoints(obs_shared);
        last_state_update_time_ = current_time;
    }

    /* 4. voxelmap update*/
    if (eskf_update) {
        for (size_t i = 0; i < points_size; ++i) {
            pv_list[i].point_w = eskf_->state().rot_ * pv_list[i].point_i + eskf_->state().pos_;
            cloud_down_world_->points[idx_i + i].x = pv_list[i].point_w(0);
            cloud_down_world_->points[idx_i + i].y = pv_list[i].point_w(1);
            cloud_down_world_->points[idx_i + i].z = pv_list[i].point_w(2);
            cloud_down_world_->points[idx_i + i].intensity = 255;

            M3D rot_extR = eskf_->state().rot_ * ext_rot_;
            M3D rot_crossmat = eskf_->state().rot_ * pv_list[i].point_crossmat;
            pv_list[i].var = rot_extR * pv_list[i].body_var * rot_extR.transpose() +
                             rot_crossmat * eskf_->cov().block<3, 3>(0, 0) * rot_crossmat.transpose() +
                             eskf_->cov().block<3, 3>(3, 3);
        }
    }
    map_manager_->UpdateVoxelMap(pv_list);

    return effect_num > 0;
}

bool RosInterface::predictUpdateImu(const sensor_msgs::ImuPtr& imu) {
    double current_time = imu->header.stamp.toSec();
    double dt_cov = current_time - last_state_update_time_;
    eskf_->predict(dt_cov, false, true);
    double dt = current_time - last_state_predict_time_;
    eskf_->predict(dt, true, false);
    last_state_predict_time_ = current_time;

    ObsShared obs_shared;
    obs_shared.ki_R.resize(6);
    obs_shared.ki_z.resize(6);
    Vec3D imu_acc(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
    Vec3D imu_gyr(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
    obs_shared.ki_z.block<3, 1>(0, 0) = (gravity_ / acc_norm_) * imu_acc - eskf_->state().imu_a_ - eskf_->state().ba_;
    obs_shared.ki_z.block<3, 1>(3, 0) = imu_gyr - eskf_->state().imu_w_ - eskf_->state().bw_;

    obs_shared.ki_R << eskf_->config().imu_acc_meas_noise, eskf_->config().imu_acc_meas_noise,
        eskf_->config().imu_acc_z_meas_noise, eskf_->config().imu_gyr_meas_noise, eskf_->config().imu_gyr_meas_noise,
        eskf_->config().imu_gyr_meas_noise;

    eskf_->updateByImu(obs_shared);
    last_state_update_time_ = current_time;
    return true;
}

bool RosInterface::predictUpdateKinImu(const common::KinImuMeas& kin_imu) {
    double current_time = kin_imu.time_stamp_;
    double dt_cov = current_time - last_state_update_time_;
    eskf_->predict(dt_cov, false, true);
    double dt = current_time - last_state_predict_time_;
    eskf_->predict(dt, true, false);
    last_state_predict_time_ = current_time;

    int contact_nums = 0;
    for (int i = 0; i < 4; ++i) {
        if (kin_imu.contact_[i]) { contact_nums++; }
    }

    ObsShared obs_shared;
    obs_shared.ki_R.resize(6 + 3 * contact_nums);
    obs_shared.ki_z.resize(6 + 3 * contact_nums);
    obs_shared.ki_h.resize(6 + 3 * contact_nums, DIM_STATE);
    obs_shared.ki_h.setZero();

    obs_shared.ki_h.block<6, 6>(0, 9) = Eigen::Matrix<double, 6, 6>::Identity();
    obs_shared.ki_h.block<6, 6>(0, 18) = Eigen::Matrix<double, 6, 6>::Identity();
    Vec3D imu_acc(kin_imu.acc_[0], kin_imu.acc_[1], kin_imu.acc_[2]);
    Vec3D imu_gyr(kin_imu.gyr_[0], kin_imu.gyr_[1], kin_imu.gyr_[2]);
    obs_shared.ki_z.block<3, 1>(0, 0) = (gravity_ / acc_norm_) * imu_acc - eskf_->state().imu_a_ - eskf_->state().ba_;
    obs_shared.ki_z.block<3, 1>(3, 0) = imu_gyr - eskf_->state().imu_w_ - eskf_->state().bw_;

    obs_shared.ki_R.block<6, 1>(0, 0) << eskf_->config().imu_acc_meas_noise, eskf_->config().imu_acc_meas_noise,
        eskf_->config().imu_acc_z_meas_noise, eskf_->config().imu_gyr_meas_noise, eskf_->config().imu_gyr_meas_noise,
        eskf_->config().imu_gyr_meas_noise;

    int idx = 0;
    Mat3D w_skew = SKEW_SYM_MATRIX(eskf_->state().imu_w_);
    for (int i = 0; i < 4; ++i) {
        if (kin_imu.contact_[i]) {
            Vec3D foot_pos(kin_imu.foot_pos_[i][0], kin_imu.foot_pos_[i][1], kin_imu.foot_pos_[i][2]);
            Vec3D foot_vel(kin_imu.foot_vel_[i][0], kin_imu.foot_vel_[i][1], kin_imu.foot_vel_[i][2]);

            Vec3D w_skew_pos_vel = w_skew * foot_pos + foot_vel;

            obs_shared.ki_h.block<3, 3>(6 + 3 * idx, 0) = -eskf_->state().rot_ * SKEW_SYM_MATRIX(w_skew_pos_vel);
            obs_shared.ki_h.block<3, 3>(6 + 3 * idx, 6) = Mat3D::Identity();
            obs_shared.ki_h.block<3, 3>(6 + 3 * idx, 21) = -eskf_->state().rot_ * SKEW_SYM_MATRIX(foot_pos);

            obs_shared.ki_z.block<3, 1>(6 + 3 * idx, 0) = -eskf_->state().vel_ - eskf_->state().rot_ * w_skew_pos_vel;

            obs_shared.ki_R.block<3, 1>(6 + 3 * idx, 0) << eskf_->config().kin_meas_noise,
                eskf_->config().kin_meas_noise, eskf_->config().kin_meas_noise;
            idx++;
        }
    }

    eskf_->updateByKinImu(obs_shared);
    last_state_update_time_ = current_time;
    return true;
}

void RosInterface::runReset() {
    cloud_raw_.reset(new PointCloudType());
    cloud_down_body_.reset(new PointCloudType());
    cloud_down_world_.reset(new PointCloudType());

    success_pts_size = 0;
}

void RosInterface::run() {
    if (!this->syncPackage()) return;
    this->runReset();

    cloud_raw_ = measure_.lidar_scan_.cloud_;
    auto& imus = measure_.imus_;
    auto& kin_imus = measure_.kin_imus_;
    double begin_time = measure_.lidar_scan_.lidar_begin_time_;
    double end_time = measure_.lidar_scan_.lidar_end_time_;

    if (cloud_raw_->points.empty() || (options::kImuUse && imus.empty()) || (!options::kImuUse && kin_imus.empty())) {
        LOG(WARNING) << "Data packet is not ready";
        return;
    }

    /* initialization */
    if (init_flag_) {
        state_initial_->processing(measure_, *eskf_);

        this->cloudLidarToWorld(cloud_raw_, cloud_down_world_);
        map_manager_->feats_down_body_ = cloud_raw_;
        map_manager_->feats_down_world_ = cloud_down_world_;
        map_manager_->BuildVoxelMap(eskf_->state().rot_, eskf_->cov().block<3, 3>(0, 0),
                                    eskf_->cov().block<3, 3>(3, 3));

        auto gravity = eskf_->state().grav_;
        auto bw = eskf_->state().bw_;
        LOG(INFO) << "Initialization is finished";
        LOG(INFO) << "Gravity is initialized to " << std::fixed << std::setprecision(3) << gravity(0) << " "
                  << gravity(1) << " " << gravity(2);
        LOG(INFO) << "IMU bw is initialized to " << bw(0) << " " << bw(1) << " " << bw(2);

        init_flag_ = false;
        acc_norm_ = state_initial_->getAccNorm();
        last_state_predict_time_ = end_time;
        last_state_update_time_ = end_time;
        return;
    }

    /* downsampling */
    Timer::measure("Downsampling", [&, this]() {
        // voxel_grid_->filter(cloud_raw_, cloud_down_body_);
        voxel_grid_.setInputCloud(cloud_raw_);
        voxel_grid_.filter(*cloud_down_body_);
    });

    Timer::measure("State predict/update & Map update", [&, this]() {
        /* pointcloud sort*/
        std::sort(cloud_down_body_->points.begin(), cloud_down_body_->points.end(), time_list);

        /* predict and update (point/imu/kin.)*/
        // points: [idx_i, idx_j)  left close right open, time compressing
        const auto& pts = cloud_down_body_->points;
        size_t pts_size = pts.size();
        size_t idx_i = 0;
        cloud_down_world_->points.resize(pts_size);
        while (idx_i < pts_size) {
            double cur_point_time = begin_time + pts[idx_i].curvature;
            size_t idx_j = idx_i + 1;
            while (idx_j < pts_size && pts[idx_i].curvature == pts[idx_j].curvature) { idx_j++; }

            if (options::kImuUse) {
                while (!imus.empty() && imus.front()->header.stamp.toSec() < cur_point_time) {
                    /* process imu*/
                    this->predictUpdateImu(imus.front());
                    imus.pop_front();
                }
            } else {
                while (!kin_imus.empty() && kin_imus.front().time_stamp_ < cur_point_time) {
                    /* process kin imu*/
                    this->predictUpdateKinImu(kin_imus.front());
                    kin_imus.pop_front();
                }
            }

            /* process points*/
            this->predictUpdatePoint(cur_point_time, idx_i, idx_j);

            idx_i = idx_j;
        }
    });
    // static int countt = 0;
    // if(!(countt++ % 10)) std::cout << eskf_->cov().block<6, 6>(0, 0) << std::endl << std::endl;

    LOG(INFO) << "pcl raw size:  " << cloud_raw_->points.size()
              << "  pcl down size: " << cloud_down_body_->points.size();
    LOG(INFO) << "useful pcl percent :  " << 100 * ((double)(success_pts_size) / cloud_down_body_->points.size())
              << " %";

    this->publishOdomTFPath(end_time);
    this->publishPointcloudWorld(end_time);

    return;
}

}  // namespace legkilo
