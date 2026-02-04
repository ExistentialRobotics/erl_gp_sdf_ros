#include "erl_common/block_timer.hpp"
#include "erl_common/eigen.hpp"
#include "erl_common/yaml.hpp"
#include "erl_geometry/depth_frame_3d.hpp"
#include "erl_geometry/lidar_frame_2d.hpp"
#include "erl_geometry/lidar_frame_3d.hpp"
#include "erl_geometry_msgs/ros1/occupancy_tree_msg.hpp"
#include "erl_gp_sdf/bayesian_hilbert_surface_mapping.hpp"
#include "erl_gp_sdf/gp_occ_surface_mapping.hpp"
#include "erl_gp_sdf/gp_sdf_mapping.hpp"
#include "erl_gp_sdf_msgs/SaveMap.h"
#include "erl_gp_sdf_msgs/SdfQuery.h"

#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <open3d/geometry/LineSet.h>
#include <open3d/geometry/TriangleMesh.h>
#include <open3d/io/LineSetIO.h>
#include <open3d/io/TriangleMeshIO.h>
#include <ros/ros.h>
#include <ros/service_server.h>
#include <ros/spinner.h>
#include <rviz/default_plugin/point_cloud_transformers.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

using namespace erl::common;

enum OdomType {
    Odometry = 0,
    TransformStamped = 1,
};

ERL_REFLECT_ENUM_SCHEMA(
    OdomType,
    2,
    ERL_REFLECT_ENUM_MEMBER("odometry", OdomType::Odometry),
    ERL_REFLECT_ENUM_MEMBER("transform_stamped", OdomType::TransformStamped));
ERL_PARSE_ENUM(OdomType, 2);

enum class ScanType {
    Laser = 0,
    PointCloud = 1,
    Depth = 2,
};

ERL_REFLECT_ENUM_SCHEMA(
    ScanType,
    3,
    ERL_REFLECT_ENUM_MEMBER("laser", ScanType::Laser),
    ERL_REFLECT_ENUM_MEMBER("point_cloud", ScanType::PointCloud),
    ERL_REFLECT_ENUM_MEMBER("depth", ScanType::Depth));
ERL_PARSE_ENUM(ScanType, 3);

struct SdfMappingNodeConfig : public Yamlable<SdfMappingNodeConfig> {
    // setting class for the surface mapping. For example, to use
    // erl::gp_sdf::GpOccSurfaceMapping<float, 2>, you should use its setting
    // class erl::gp_sdf::GpOccSurfaceMapping<float, 2>::Setting.
    std::string surface_mapping_setting_type = "";
    // path to the yaml file for the surface mapping setting
    std::string surface_mapping_setting_file = "";
    // surface mapping class type. For example,
    // erl::gp_sdf::GpOccSurfaceMapping<float, 2>.
    std::string surface_mapping_type = "";
    // path to the yaml file for the SDF mapping setting
    std::string sdf_mapping_setting_file = "";
    // whether to use the odometry topic to get the sensor pose
    bool use_odom = false;
    // name of the odometry topic
    std::string odom_topic = "/jackal_velocity_controller/odom";
    // can be "odometry" or "transform_stamped"
    OdomType odom_msg_type = OdomType::Odometry;
    // size of the odometry queue
    int odom_queue_size = 100;
    // name of the world frame
    std::string world_frame = "map";
    // name of the sensor frame
    std::string sensor_frame = "front_laser";
    // name of the scan topic
    std::string scan_topic = "/front/scan";
    // can be "laser", "point_cloud", or "depth"
    ScanType scan_type = ScanType::Laser;
    // frame class of the scan. e.g. erl::geometry::LidarFrame3D<float>,
    // erl::geometry::DepthFrame3D<float> for 3D scans. For 2D scans, the only
    // option is erl::geometry::LidarFrame2D<float> or
    // erl::geometry::LidarFrame2D<double>.
    std::string scan_frame_type = "";
    // path to the yaml file for the scan frame setting
    std::string scan_frame_setting_file = "";
    // if scan_stride > 1, the scan will be downsampled by this factor.
    int scan_stride = 1;
    // if true, convert the scan to points when the scan is not a point cloud.
    bool convert_scan_to_points = false;
    // if the scan data is in the local frame, set this to true.
    bool scan_in_local_frame = false;
    // scale for depth image, 0.001 converts mm to m.
    float depth_scale = 0.001f;
    // if true, publish the occupancy tree used by the surface mapping.
    bool publish_tree = false;
    // if true, use binary format to publish the occupancy tree, which makes the
    // message smaller.
    bool publish_tree_binary = true;
    // frequency to publish the occupancy tree
    double publish_tree_frequency = 5.0;
    // the topic to publish the occupancy tree
    std::string tree_topic = "surface_mapping_tree";
    // if true, publish the surface points used by the sdf mapping.
    bool publish_surface_points = false;
    // frequency to publish the surface points
    double publish_surface_points_frequency = 5.0;
    // the topic to publish the surface points
    std::string surface_points_topic = "surface_points";
    // the topic to publish the update time
    std::string update_time_topic = "update_time";
    // the topic to publish the query time
    std::string query_time_topic = "query_time";
    // the service to query the SDF value
    std::string sdf_query_service = "sdf_query";
    // the service to save the map
    std::string save_map_service = "save_map";
    // the service to load the map
    std::string load_map_service = "load_map";
    // the service to save mesh
    std::string save_mesh_service = "save_mesh";

    ERL_REFLECT_SCHEMA(
        SdfMappingNodeConfig,
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, surface_mapping_setting_type),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, surface_mapping_setting_file),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, surface_mapping_type),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, sdf_mapping_setting_file),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, use_odom),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, odom_topic),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, odom_msg_type),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, odom_queue_size),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, world_frame),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, sensor_frame),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, scan_topic),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, scan_type),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, scan_frame_type),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, scan_frame_setting_file),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, scan_stride),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, convert_scan_to_points),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, scan_in_local_frame),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, depth_scale),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, publish_tree),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, publish_tree_binary),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, publish_tree_frequency),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, tree_topic),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, publish_surface_points),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, publish_surface_points_frequency),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, surface_points_topic),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, update_time_topic),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, query_time_topic),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, sdf_query_service),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, save_map_service),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, load_map_service),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, save_mesh_service));

    bool
    PostDeserialization() override {
        if (surface_mapping_setting_type.empty()) {
            ROS_WARN("You must set ~surface_mapping_setting_type");
            return false;
        }
        if (surface_mapping_setting_file.empty()) {
            ROS_WARN("You must set ~surface_mapping_config");
            return false;
        }
        if (!std::filesystem::exists(surface_mapping_setting_file)) {
            ROS_WARN(
                "Surface mapping setting file %s does not exist",
                surface_mapping_setting_file.c_str());
            return false;
        }
        if (surface_mapping_type.empty()) {
            ROS_WARN("You must set ~surface_mapping_type");
            return false;
        }
        if (sdf_mapping_setting_file.empty()) {
            ROS_WARN("You must set ~sdf_mapping_config");
            return false;
        }
        if (!std::filesystem::exists(sdf_mapping_setting_file)) {
            ROS_WARN(
                "SDF mapping setting file %s does not exist",
                sdf_mapping_setting_file.c_str());
            return false;
        }
        if (use_odom && odom_topic.empty()) {
            ROS_WARN("Odometry topic is empty but use_odom is true");
            return false;
        }
        if (odom_queue_size <= 0) {
            ROS_WARN("Odometry queue size must be positive");
            return false;
        }
        if (world_frame.empty()) {
            ROS_WARN("World frame is empty");
            return false;
        }
        if (!use_odom && sensor_frame.empty()) {
            ROS_WARN("Sensor frame is empty but use_odom is false");
            return false;
        }
        if (scan_topic.empty()) {
            ROS_WARN("Scan topic is empty");
            return false;
        }
        if (scan_stride <= 0) {
            ROS_WARN("Scan stride must be positive");
            return false;
        }
        if (convert_scan_to_points) {
            if (scan_frame_setting_file.empty()) {
                ROS_WARN("For scan conversion, scan_frame_setting_file must be set.");
                return false;
            }
            if (!std::filesystem::exists(scan_frame_setting_file)) {
                ROS_WARN(
                    "Scan frame setting file %s does not exist.",
                    scan_frame_setting_file.c_str());
                return false;
            }
        }
        if (publish_tree && tree_topic.empty()) {
            ROS_WARN("Publish tree topic is empty but publish_tree is true");
            return false;
        }
        if (publish_tree && publish_tree_frequency <= 0.0) {
            ROS_WARN("Publish tree frequency must be positive");
            return false;
        }
        if (publish_surface_points && surface_points_topic.empty()) {
            ROS_WARN(
                "Publish surface points topic is empty but "
                "publish_surface_points is true");
            return false;
        }
        if (publish_surface_points && publish_surface_points_frequency <= 0.0) {
            ROS_WARN("Publish surface points frequency must be positive");
            return false;
        }
        if (update_time_topic.empty()) {
            ROS_WARN("update_time_topic is empty");
            return false;
        }
        if (query_time_topic.empty()) {
            ROS_WARN("query_time_topic is empty");
            return false;
        }
        if (sdf_query_service.empty()) {
            ROS_WARN("sdf_query_service is empty");
            return false;
        }
        if (save_map_service.empty()) {
            ROS_WARN("save_map_service is empty");
            return false;
        }
        if (load_map_service.empty()) {
            ROS_WARN("load_map_service is empty");
            return false;
        }
        if (save_mesh_service.empty()) {
            ROS_WARN("save_mesh_service is empty");
            return false;
        }
        return true;
    }
};

template<typename Dtype, int Dim>
class SdfMappingNode {
    using AbstractSurfaceMapping = erl::gp_sdf::AbstractSurfaceMapping<Dtype, Dim>;
    using GpSdfMapping = erl::gp_sdf::GpSdfMapping<Dtype, Dim>;
    using GpOccSurfaceMapping = erl::gp_sdf::GpOccSurfaceMapping<Dtype, Dim>;
    using BayesianHilbertSurfaceMapping = erl::gp_sdf::BayesianHilbertSurfaceMapping<Dtype, Dim>;
    using Tree = std::conditional_t<
        Dim == 2,
        erl::geometry::OccupancyQuadtree<Dtype>,
        erl::geometry::OccupancyOctree<Dtype>>;
    using Rotation = typename GpSdfMapping::Rotation;
    using Translation = typename GpSdfMapping::Translation;
    using Variances = typename GpSdfMapping::Variances;
    using Covariances = typename GpSdfMapping::Covariances;
    using VectorD = typename GpSdfMapping::VectorD;
    using MatrixDX = typename GpSdfMapping::MatrixDX;
    using Matrix4 = Eigen::Matrix4<Dtype>;
    using Matrix3 = Eigen::Matrix3<Dtype>;
    using Vector3 = Eigen::Vector3<Dtype>;
    using Matrix2 = Eigen::Matrix2<Dtype>;
    using Vector2 = Eigen::Vector2<Dtype>;
    using VectorX = Eigen::VectorX<Dtype>;
    using MatrixX = Eigen::MatrixX<Dtype>;
    using Matrix3X = Eigen::Matrix3X<Dtype>;
    using RangeSensorFrame3D = erl::geometry::RangeSensorFrame3D<Dtype>;
    using LidarFrame2D = erl::geometry::LidarFrame2D<Dtype>;
    using LidarFrame3D = erl::geometry::LidarFrame3D<Dtype>;
    using DepthFrame3D = erl::geometry::DepthFrame3D<Dtype>;

    SdfMappingNodeConfig m_setting_;
    ros::NodeHandle m_nh_;
    ros::Subscriber m_sub_odom_;
    ros::Subscriber m_sub_scan_;
    ros::ServiceServer m_srv_query_sdf_;
    ros::ServiceServer m_srv_load_map_;
    ros::ServiceServer m_srv_save_map_;
    ros::ServiceServer m_srv_save_mesh_;
    ros::Publisher m_pub_tree_;
    ros::Publisher m_pub_surface_points_;
    ros::Publisher m_pub_update_time_;
    ros::Publisher m_pub_query_time_;
    ros::Timer m_pub_tree_timer_;
    ros::Timer m_pub_surface_points_timer_;
    erl_geometry_msgs::OccupancyTreeMsg m_msg_tree_;
    sensor_msgs::PointCloud2 m_msg_surface_points_;
    std_msgs::Float64 m_msg_update_time_;
    std_msgs::Float64 m_msg_query_time_;

    std::shared_ptr<YamlableBase> m_surface_mapping_cfg_ = nullptr;
    std::shared_ptr<AbstractSurfaceMapping> m_surface_mapping_ = nullptr;
    std::shared_ptr<typename GpSdfMapping::Setting> m_sdf_mapping_cfg_ = nullptr;
    std::shared_ptr<GpSdfMapping> m_sdf_mapping_ = nullptr;
    std::shared_ptr<const Tree> m_tree_ = nullptr;  // used to store the occupancy tree

    // for the sensor pose

    std::mutex m_odom_queue_lock_;
    std::vector<geometry_msgs::TransformStamped> m_odom_queue_{};
    int m_odom_queue_head_ = -1;
    tf2_ros::Buffer m_tf_buffer_;
    tf2_ros::TransformListener m_tf_listener_{m_tf_buffer_};

    // for the scan data

    sensor_msgs::LaserScan::ConstPtr m_lidar_scan_2d_ = nullptr;
    sensor_msgs::PointCloud2::ConstPtr m_lidar_scan_3d_ = nullptr;
    sensor_msgs::Image::ConstPtr m_depth_image_ = nullptr;

    std::shared_ptr<LidarFrame2D> m_scan_frame_2d_ = nullptr;
    std::shared_ptr<RangeSensorFrame3D> m_scan_frame_3d_ = nullptr;

public:
    SdfMappingNode(ros::NodeHandle &nh)
        : m_nh_(nh) {
        if (!LoadParameters()) {
            ROS_FATAL("Failed to load parameters");
            ros::shutdown();
            return;
        }

        auto &setting_factory = YamlableBase::Factory::GetInstance();

        // load the surface mapping config
        m_surface_mapping_cfg_ = setting_factory.Create(m_setting_.surface_mapping_setting_type);
        if (!m_surface_mapping_cfg_) {
            ROS_FATAL("Failed to create surface mapping config");
            ros::shutdown();
            return;
        }
        try {
            if (!m_surface_mapping_cfg_->FromYamlFile(m_setting_.surface_mapping_setting_file)) {
                ROS_FATAL(
                    "Failed to load %s with surface mapping type %s",
                    m_setting_.surface_mapping_setting_file.c_str(),
                    m_setting_.surface_mapping_type.c_str());
                ros::shutdown();
                return;
            }
        } catch (const std::exception &e) {
            ROS_FATAL(
                "Failed to parse %s with surface mapping type %s: %s",
                m_setting_.surface_mapping_setting_file.c_str(),
                m_setting_.surface_mapping_type.c_str(),
                e.what());
            ros::shutdown();
            return;
        }

        // load the sdf mapping config
        m_sdf_mapping_cfg_ = std::make_shared<typename GpSdfMapping::Setting>();
        try {
            if (!m_sdf_mapping_cfg_->FromYamlFile(m_setting_.sdf_mapping_setting_file)) {
                ROS_FATAL("Failed to load %s", m_setting_.sdf_mapping_setting_file.c_str());
                ros::shutdown();
                return;
            }
        } catch (const std::exception &e) {
            ROS_FATAL(
                "Failed to parse %s: %s",
                m_setting_.sdf_mapping_setting_file.c_str(),
                e.what());
            ros::shutdown();
            return;
        }

        // create the surface mapping
        m_surface_mapping_ =
            AbstractSurfaceMapping::Create(m_setting_.surface_mapping_type, m_surface_mapping_cfg_);
        m_sdf_mapping_ = std::make_shared<GpSdfMapping>(m_sdf_mapping_cfg_, m_surface_mapping_);
        ROS_INFO("Created surface mapping of type %s", m_setting_.surface_mapping_type.c_str());
        ROS_INFO("Surface mapping config:\n%s", m_surface_mapping_cfg_->AsYamlString().c_str());
        ROS_INFO("SDF mapping config:\n%s", m_sdf_mapping_cfg_->AsYamlString().c_str());

        if (m_setting_.use_odom) {
            switch (m_setting_.odom_msg_type) {
                case OdomType::Odometry:
                    ROS_INFO(
                        "Subscribing to %s as nav_msgs/Odometry",
                        m_setting_.odom_topic.c_str());
                    m_sub_odom_ = m_nh_.subscribe(
                        m_setting_.odom_topic,
                        1,
                        &SdfMappingNode::CallbackOdomOdometry,
                        this);
                    break;
                case OdomType::TransformStamped:
                    ROS_INFO(
                        "Subscribing to %s as geometry_msgs/TransformStamped",
                        m_setting_.odom_topic.c_str());
                    m_sub_odom_ = m_nh_.subscribe(
                        m_setting_.odom_topic,
                        1,
                        &SdfMappingNode::CallbackOdomTransformStamped,
                        this);
                    break;
                default:
                    ROS_FATAL(
                        "Invalid odometry message type: %d",
                        static_cast<int>(m_setting_.odom_msg_type));
                    ros::shutdown();
                    return;
            }
            m_odom_queue_.reserve(m_setting_.odom_queue_size);
        }

        switch (m_setting_.scan_type) {
            case ScanType::Laser:
                ROS_INFO("Subscribing to %s as laser scan", m_setting_.scan_topic.c_str());
                m_sub_scan_ = m_nh_.subscribe(
                    m_setting_.scan_topic,
                    10,
                    &SdfMappingNode::CallbackLaserScan,
                    this);
                break;
            case ScanType::PointCloud:
                ROS_INFO("Subscribing to %s as point cloud", m_setting_.scan_topic.c_str());
                m_sub_scan_ = m_nh_.subscribe(
                    m_setting_.scan_topic,
                    10,
                    &SdfMappingNode::CallbackPointCloud2,
                    this);
                break;
            case ScanType::Depth:
                ROS_INFO("Subscribing to %s as depth image", m_setting_.scan_topic.c_str());
                m_sub_scan_ = m_nh_.subscribe(
                    m_setting_.scan_topic,
                    10,
                    &SdfMappingNode::CallbackDepthImage,
                    this);
                break;
        }

        if (m_setting_.convert_scan_to_points) {
            if (Dim == 2) {
                auto frame_setting = std::make_shared<typename LidarFrame2D::Setting>();
                try {
                    if (!frame_setting->FromYamlFile(m_setting_.scan_frame_setting_file)) {
                        ROS_FATAL(
                            "Failed to load %s with frame type %s",
                            m_setting_.scan_frame_setting_file.c_str(),
                            m_setting_.scan_frame_type.c_str());
                        ros::shutdown();
                        return;
                    }
                } catch (const std::exception &e) {
                    ROS_FATAL(
                        "Failed to parse %s with frame type %s: %s",
                        m_setting_.scan_frame_setting_file.c_str(),
                        m_setting_.scan_frame_type.c_str(),
                        e.what());
                    ros::shutdown();
                    return;
                }
                if (m_setting_.scan_stride > 1) {
                    frame_setting->Resize(1.0f / static_cast<Dtype>(m_setting_.scan_stride));
                }
                m_scan_frame_2d_ = std::make_shared<LidarFrame2D>(frame_setting);
                ROS_INFO(
                    "Created scan frame of type %s with setting:\n%s",
                    m_setting_.scan_frame_type.c_str(),
                    frame_setting->AsYamlString().c_str());
            } else if (m_setting_.scan_frame_type == type_name<LidarFrame3D>()) {
                auto frame_setting = std::make_shared<typename LidarFrame3D::Setting>();
                try {
                    if (!frame_setting->FromYamlFile(m_setting_.scan_frame_setting_file)) {
                        ROS_FATAL(
                            "Failed to load %s with frame type %s",
                            m_setting_.scan_frame_setting_file.c_str(),
                            m_setting_.scan_frame_type.c_str());
                        ros::shutdown();
                        return;
                    }
                } catch (const std::exception &e) {
                    ROS_FATAL(
                        "Failed to parse %s with frame type %s: %s",
                        m_setting_.scan_frame_setting_file.c_str(),
                        m_setting_.scan_frame_type.c_str(),
                        e.what());
                    ros::shutdown();
                    return;
                }
                if (m_setting_.scan_stride > 1) {
                    frame_setting->Resize(1.0f / static_cast<Dtype>(m_setting_.scan_stride));
                }
                m_scan_frame_3d_ = std::make_shared<LidarFrame3D>(frame_setting);
                ROS_INFO(
                    "Created scan frame of type %s with setting:\n%s",
                    m_setting_.scan_frame_type.c_str(),
                    frame_setting->AsYamlString().c_str());
            } else if (m_setting_.scan_frame_type == type_name<DepthFrame3D>()) {
                auto frame_setting = std::make_shared<typename DepthFrame3D::Setting>();
                try {
                    if (!frame_setting->FromYamlFile(m_setting_.scan_frame_setting_file)) {
                        ROS_FATAL(
                            "Failed to load %s with frame type %s",
                            m_setting_.scan_frame_setting_file.c_str(),
                            m_setting_.scan_frame_type.c_str());
                        ros::shutdown();
                        return;
                    }
                } catch (const std::exception &e) {
                    ROS_FATAL(
                        "Failed to parse %s with frame type %s: %s",
                        m_setting_.scan_frame_setting_file.c_str(),
                        m_setting_.scan_frame_type.c_str(),
                        e.what());
                    ros::shutdown();
                    return;
                }
                if (m_setting_.scan_stride > 1) {
                    frame_setting->Resize(1.0f / static_cast<Dtype>(m_setting_.scan_stride));
                }
                m_scan_frame_3d_ = std::make_shared<DepthFrame3D>(frame_setting);
                ROS_INFO(
                    "Created scan frame of type %s with setting:\n%s",
                    m_setting_.scan_frame_type.c_str(),
                    frame_setting->AsYamlString().c_str());
            } else {
                ROS_FATAL("Invalid scan frame type: %s", m_setting_.scan_frame_type.c_str());
                ros::shutdown();
                return;
            }
        }

        // advertise the service to query the SDF mapping
        m_srv_query_sdf_ = m_nh_.advertiseService(
            m_setting_.sdf_query_service,
            &SdfMappingNode::CallbackSdfQuery,
            this);
        m_srv_load_map_ = m_nh_.advertiseService(
            m_setting_.load_map_service,
            &SdfMappingNode::CallbackLoadMap,
            this);
        m_srv_save_map_ = m_nh_.advertiseService(
            m_setting_.save_map_service,
            &SdfMappingNode::CallbackSaveMap,
            this);
        m_srv_save_mesh_ = m_nh_.advertiseService(
            m_setting_.save_mesh_service,
            &SdfMappingNode::CallbackSaveMesh,
            this);

        // publish the occupancy tree used by the surface mapping
        if (m_setting_.publish_tree) {
            if (!TryToGetSurfaceMappingTree()) {
                ROS_FATAL("Failed to get surface mapping tree");
                ros::shutdown();
                return;
            }
            m_pub_tree_ = m_nh_.advertise<erl_geometry_msgs::OccupancyTreeMsg>(
                m_setting_.tree_topic,
                1,
                true);
            m_pub_tree_timer_ = m_nh_.createTimer(
                ros::Duration(1.0 / m_setting_.publish_tree_frequency),
                &SdfMappingNode::CallbackPublishTree,
                this);
            m_msg_tree_.header.frame_id = m_setting_.world_frame;
            m_msg_tree_.header.seq = -1;
        }

        // publish the surface points used by the sdf mapping
        if (m_setting_.publish_surface_points) {
            m_pub_surface_points_ =
                m_nh_.advertise<sensor_msgs::PointCloud2>(m_setting_.surface_points_topic, 1, true);
            m_pub_surface_points_timer_ = m_nh_.createTimer(
                ros::Duration(1.0 / m_setting_.publish_surface_points_frequency),
                &SdfMappingNode::CallbackPublishSurfacePoints,
                this);
            m_msg_surface_points_.header.frame_id = m_setting_.world_frame;
            m_msg_surface_points_.header.seq = -1;
            m_msg_surface_points_.fields.resize(8);
            static const char *kSurfacePointsFieldNames[] = {
                "x",
                "y",
                "z",
                "normal_x",
                "normal_y",
                "normal_z",
                "var_position",
                "var_normal",
            };
            for (int i = 0; i < 8; ++i) {
                m_msg_surface_points_.fields[i].name = kSurfacePointsFieldNames[i];
                m_msg_surface_points_.fields[i].offset = i * 4;  // each field is 4 bytes (float)
                m_msg_surface_points_.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
            }
            m_msg_surface_points_.point_step = 32;       // 8 fields * 4 bytes each
            m_msg_surface_points_.is_bigendian = false;  // little-endian
            m_msg_surface_points_.is_dense = false;      // there may be NaN values in the normals
            m_msg_surface_points_.height = 1;            // unorganized point cloud
        }

        m_pub_update_time_ =
            m_nh_.advertise<std_msgs::Float64>(m_setting_.update_time_topic, 1, true);
        m_pub_query_time_ =
            m_nh_.advertise<std_msgs::Float64>(m_setting_.query_time_topic, 1, true);
        m_msg_update_time_.data = 0.0;
        m_msg_query_time_.data = 0.0;

        ROS_INFO("SdfMappingNode is ready. Waiting for scans + queries...");
    }

private:
    bool
    LoadParameters() {
        if (!m_setting_.LoadFromRos1(m_nh_, "")) { return false; }

        // more checks
        if (m_setting_.convert_scan_to_points) {
            if (Dim == 2) {
                ROS_WARN_COND(
                    m_setting_.scan_frame_type != type_name<LidarFrame2D>(),
                    "For 2D scans, scan_frame_type is %s but must be %s.",
                    m_setting_.scan_frame_type.c_str(),
                    type_name<LidarFrame2D>().c_str());
            } else {
                if (m_setting_.scan_frame_type != type_name<LidarFrame3D>() &&
                    m_setting_.scan_frame_type != type_name<DepthFrame3D>()) {
                    ROS_WARN(
                        "For 3D scans, scan_frame_type must be %s or %s. Not %s.",
                        type_name<LidarFrame3D>().c_str(),
                        type_name<DepthFrame3D>().c_str(),
                        m_setting_.scan_frame_type.c_str());
                    return false;
                }
            }
        }
        return true;
    }

    bool
    TryToGetSurfaceMappingTree() {
        m_tree_ = GetTreeFromGpOccSurfaceMapping();
        if (m_tree_) { return true; }
        m_tree_ = GetTreeFromBayesianHilbertSurfaceMapping();
        if (m_tree_) { return true; }
        return false;  // no valid tree found
    }

    std::shared_ptr<const Tree>
    GetTreeFromGpOccSurfaceMapping() {
        auto mapping = std::dynamic_pointer_cast<GpOccSurfaceMapping>(m_surface_mapping_);
        if (!mapping) { return nullptr; }
        return mapping->GetTree();
    }

    std::shared_ptr<const Tree>
    GetTreeFromBayesianHilbertSurfaceMapping() {
        auto mapping = std::dynamic_pointer_cast<BayesianHilbertSurfaceMapping>(m_surface_mapping_);
        if (!mapping) { return nullptr; }
        return mapping->GetTree();
    }

    // get the pose for time t
    std::tuple<bool, Rotation, Translation>
    GetSensorPose(const ros::Time &time) {
        if (m_setting_.use_odom) {
            geometry_msgs::TransformStamped transform;

            {
                std::lock_guard<std::mutex> lock(m_odom_queue_lock_);
                // get the latest odometry message
                const int &head = m_odom_queue_head_;
                if (head < 0) {
                    ROS_WARN("No odometry message available");
                    return {false, {}, {}};
                }
                geometry_msgs::TransformStamped *transform_ptr = nullptr;
                for (int i = head; i >= 0; --i) {
                    if (m_odom_queue_[i].header.stamp <= time) {
                        transform_ptr = &m_odom_queue_[i];
                        break;
                    }
                }
                if (!transform_ptr) {  // search older messages
                    const int size = static_cast<int>(m_odom_queue_.size());
                    for (int i = size - 1; i > head; --i) {
                        if (m_odom_queue_[i].header.stamp <= time) {
                            transform_ptr = &m_odom_queue_[i];
                            break;
                        }
                    }
                }
                if (!transform_ptr) {
                    ROS_WARN("No odometry message available for time %f", time.toSec());
                    return {false, {}, {}};
                }
                transform = *transform_ptr;  // copy the transform
            }

            if (transform.child_frame_id != m_setting_.sensor_frame) {
                geometry_msgs::TransformStamped tf_child_to_sensor;
                try {
                    tf_child_to_sensor = m_tf_buffer_.lookupTransform(
                        transform.child_frame_id,
                        m_setting_.sensor_frame,
                        transform.header.stamp,
                        ros::Duration(0.5));
                } catch (tf2::LookupException &ex) {
                    ROS_WARN(
                        "Failed to lookup transform from %s to %s: %s",
                        transform.child_frame_id.c_str(),
                        m_setting_.sensor_frame.c_str(),
                        ex.what());
                    return {false, {}, {}};
                }
                tf2::doTransform(tf_child_to_sensor, transform, transform);
            }

            auto &pose = transform.transform;
            // erase the dimension of rotation and translation temporarily
            MatrixX rotation;
            VectorX translation;
            if (Dim == 2) {
                const double yaw = tf::getYaw(pose.rotation);
                rotation = Eigen::Rotation2D<Dtype>(yaw).toRotationMatrix();
                translation = Eigen::Vector2<Dtype>(pose.translation.x, pose.translation.y);
            } else {
                rotation = Eigen::Quaternion<Dtype>(
                               pose.rotation.w,
                               pose.rotation.x,
                               pose.rotation.y,
                               pose.rotation.z)
                               .toRotationMatrix();
                translation = Vector3(pose.translation.x, pose.translation.y, pose.translation.z);
            }
            // recover the dimension of rotation and translation
            return {true, Rotation(rotation), Translation(translation)};
        }
        // get the latest transform from the tf buffer
        geometry_msgs::TransformStamped transform_stamped;
        try {
            transform_stamped = m_tf_buffer_.lookupTransform(
                m_setting_.world_frame,
                m_setting_.sensor_frame,
                time,
                ros::Duration(5.0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN(ex.what());
            return {false, {}, {}};  // no valid transform
        }
        Matrix4 pose = tf2::transformToEigen(transform_stamped).matrix().cast<Dtype>();
        MatrixX rotation;
        VectorX translation;
        if (Dim == 2) {
            rotation = pose.template block<2, 2>(0, 0);
            translation = pose.template block<2, 1>(0, 3);
        } else {
            rotation = pose.template block<3, 3>(0, 0);
            translation = pose.template block<3, 1>(0, 3);
        }
        return {true, Rotation(rotation), Translation(translation)};
    }

    // --- callbacks to collect pose+scan and then update the map ---
    void
    CallbackOdomOdometry(const nav_msgs::Odometry::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(m_odom_queue_lock_);
        if (static_cast<int>(m_odom_queue_.size()) >= m_setting_.odom_queue_size) {
            auto &transform = m_odom_queue_[m_odom_queue_head_];
            transform.header = msg->header;
            transform.child_frame_id = msg->child_frame_id;
            transform.transform.rotation = msg->pose.pose.orientation;
            transform.transform.translation.x = msg->pose.pose.position.x;
            transform.transform.translation.y = msg->pose.pose.position.y;
            transform.transform.translation.z = msg->pose.pose.position.z;
            m_odom_queue_head_ = (m_odom_queue_head_ + 1) % m_setting_.odom_queue_size;
        } else {
            geometry_msgs::TransformStamped transform;
            transform.header = msg->header;
            transform.child_frame_id = msg->child_frame_id;
            transform.transform.rotation = msg->pose.pose.orientation;
            transform.transform.translation.x = msg->pose.pose.position.x;
            transform.transform.translation.y = msg->pose.pose.position.y;
            transform.transform.translation.z = msg->pose.pose.position.z;
            m_odom_queue_.push_back(transform);
            ++m_odom_queue_head_;
        }
    }

    void
    CallbackOdomTransformStamped(const geometry_msgs::TransformStamped::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(m_odom_queue_lock_);
        if (static_cast<int>(m_odom_queue_.size()) >= m_setting_.odom_queue_size) {
            auto &transform = m_odom_queue_[m_odom_queue_head_];
            transform.header = msg->header;
            transform.child_frame_id = msg->child_frame_id;
            transform.transform = msg->transform;
            m_odom_queue_head_ = (m_odom_queue_head_ + 1) % m_setting_.odom_queue_size;
        } else {
            m_odom_queue_.push_back(*msg);
            ++m_odom_queue_head_;
        }
    }

    void
    CallbackLaserScan(const sensor_msgs::LaserScan::ConstPtr &msg) {
        m_lidar_scan_2d_ = msg;
        TryUpdate(msg->header.stamp);
    }

    void
    CallbackPointCloud2(const sensor_msgs::PointCloud2::ConstPtr &msg) {
        m_lidar_scan_3d_ = msg;
        TryUpdate(msg->header.stamp);
    }

    void
    CallbackDepthImage(const sensor_msgs::Image::ConstPtr &msg) {
        m_depth_image_ = msg;
        TryUpdate(msg->header.stamp);
    }

    bool
    GetScanFromLaserScan(MatrixX &scan) {
        if (!m_lidar_scan_2d_) {
            ROS_WARN("No laser scan data available");
            return false;
        }
        auto &scan_msg = *m_lidar_scan_2d_;
        if (scan_msg.ranges.empty()) {
            ROS_WARN("Laser scan data is empty");
            m_lidar_scan_2d_.reset();
            return false;
        }
        scan = Eigen::Map<const Eigen::VectorXf>(scan_msg.ranges.data(), scan_msg.ranges.size())
                   .cast<Dtype>();
        if (m_setting_.scan_stride > 1) {
            scan = DownsampleEigenMatrix(scan, m_setting_.scan_stride, 1);
        }
        for (long i = 0; i < scan.size(); ++i) {
            if (!std::isfinite(scan(i, 0))) { scan(i, 0) = 0.0; }  // invalid range
        }
        m_lidar_scan_2d_.reset();
        return true;
    }

    bool
    GetScanFromPointCloud2(MatrixX &scan) {
        if (!m_lidar_scan_3d_) {
            ROS_WARN("No point cloud data available");
            return false;
        }
        auto &cloud = *m_lidar_scan_3d_;
        if (cloud.fields.empty() || cloud.data.empty()) {
            ROS_WARN("Point cloud data is empty");
            m_lidar_scan_3d_.reset();
            return false;
        }
        if (cloud.data.size() != cloud.width * cloud.height * cloud.point_step) {
            ROS_WARN("Point cloud data size does not match width, height, and point step");
            m_lidar_scan_3d_.reset();
            return false;
        }
        if (cloud.row_step != cloud.width * cloud.point_step) {
            ROS_WARN("Point cloud row step does not match width and point step");
            m_lidar_scan_3d_.reset();
            return false;
        }

        // validate x, y, z are present
        const int32_t xi = rviz::findChannelIndex(m_lidar_scan_3d_, "x");
        const int32_t yi = rviz::findChannelIndex(m_lidar_scan_3d_, "y");
        const int32_t zi = rviz::findChannelIndex(m_lidar_scan_3d_, "z");
        if (xi < 0 || yi < 0 || zi < 0) {
            ROS_WARN("Point cloud does not contain x, y, z fields");
            m_lidar_scan_3d_.reset();
            return false;
        }
        // validate x, y, z fields have the same data type
        const uint8_t &xtype = cloud.fields[xi].datatype;
        const uint8_t &ytype = cloud.fields[yi].datatype;
        const uint8_t &ztype = cloud.fields[zi].datatype;
        if (xtype != ytype || xtype != ztype || ytype != ztype) {
            ROS_WARN("Point cloud x, y, z fields have different data types");
            m_lidar_scan_3d_.reset();
            return false;
        }
        const uint32_t xoff = cloud.fields[xi].offset;
        const uint32_t yoff = cloud.fields[yi].offset;
        const uint32_t zoff = cloud.fields[zi].offset;
        uint32_t point_step = cloud.point_step;
        uint32_t row_step = cloud.row_step;
        auto width = static_cast<int>(cloud.width);
        auto height = static_cast<int>(cloud.height);
        const int scan_stride = m_setting_.scan_stride;
        if (scan_stride > 1) {
            width = (width + scan_stride - 1) / scan_stride;
            height = (height + scan_stride - 1) / scan_stride;
            point_step *= scan_stride;
            row_step *= scan_stride;
        }
        scan.resize(3, width * height);
        long point_count = 0;
        if (xtype == sensor_msgs::PointField::FLOAT32) {
            for (int h = 0; h < height; ++h) {
                const uint8_t *ptr = cloud.data.data() + h * row_step;
                for (int w = 0; w < width; ++w) {
                    Dtype *p_out = scan.col(point_count).data();
                    p_out[0] = static_cast<Dtype>(*reinterpret_cast<const float *>(ptr + xoff));
                    p_out[1] = static_cast<Dtype>(*reinterpret_cast<const float *>(ptr + yoff));
                    p_out[2] = static_cast<Dtype>(*reinterpret_cast<const float *>(ptr + zoff));
                    if (std::isfinite(p_out[0]) && std::isfinite(p_out[1]) &&
                        std::isfinite(p_out[2])) {
                        ++point_count;
                    }
                    ptr += point_step;
                }
            }
        } else if (xtype == sensor_msgs::PointField::FLOAT64) {
            for (int h = 0; h < height; ++h) {
                const uint8_t *ptr = cloud.data.data() + h * row_step;
                for (int w = 0; w < width; ++w) {
                    Dtype *p_out = scan.col(point_count).data();
                    p_out[0] = static_cast<Dtype>(*reinterpret_cast<const double *>(ptr + xoff));
                    p_out[1] = static_cast<Dtype>(*reinterpret_cast<const double *>(ptr + yoff));
                    p_out[2] = static_cast<Dtype>(*reinterpret_cast<const double *>(ptr + zoff));
                    if (std::isfinite(p_out[0]) && std::isfinite(p_out[1]) &&
                        std::isfinite(p_out[2])) {
                        ++point_count;
                    }
                    ptr += point_step;
                }
            }
        } else {
            ROS_WARN("Unsupported point cloud data type %d", xtype);
            m_lidar_scan_3d_.reset();
            return false;
        }
        if (point_count == 0) {
            ROS_WARN("No valid points in point cloud");
            m_lidar_scan_3d_.reset();
            return false;
        }
        scan.conservativeResize(3, point_count);
        m_lidar_scan_3d_.reset();
        return true;
    }

    bool
    GetScanFromDepthImage(MatrixX &scan) {
        if (!m_depth_image_) {
            ROS_WARN("No depth image available");
            return false;
        }
        using namespace sensor_msgs::image_encodings;
        // row-major to column-major conversion
        if (m_depth_image_->encoding == TYPE_32FC1) {
            MatrixX depth_image = Eigen::Map<const Eigen::MatrixXf>(
                                      reinterpret_cast<const float *>(m_depth_image_->data.data()),
                                      m_depth_image_->width,
                                      m_depth_image_->height)
                                      .cast<Dtype>();
            scan = depth_image.transpose();
        } else if (m_depth_image_->encoding == TYPE_64FC1) {
            MatrixX depth_image = Eigen::Map<const Eigen::MatrixXd>(
                                      reinterpret_cast<const double *>(m_depth_image_->data.data()),
                                      m_depth_image_->width,
                                      m_depth_image_->height)
                                      .cast<Dtype>();
            scan = depth_image.transpose();
        } else if (m_depth_image_->encoding == TYPE_16UC1) {
            MatrixX depth_image =
                Eigen::Map<const Eigen::MatrixX<uint16_t>>(
                    reinterpret_cast<const uint16_t *>(m_depth_image_->data.data()),
                    m_depth_image_->width,
                    m_depth_image_->height)
                    .cast<Dtype>();
            scan = depth_image.transpose();
        } else {
            ROS_WARN("Unsupported depth encoding %s", m_depth_image_->encoding.c_str());
            m_depth_image_.reset();
            return false;
        }
        if (scan.size() > 0) {
            if (m_setting_.scan_stride > 1) {
                scan = DownsampleEigenMatrix(scan, m_setting_.scan_stride, m_setting_.scan_stride);
            }
            long cnt_valid = 0;
            Dtype *ptr = scan.data();
            for (long i = 0; i < scan.size(); ++i) {
                if (std::isfinite(ptr[i]) && ptr[i] > 0) {
                    ++cnt_valid;
                } else {
                    ptr[i] = -1.0f;  // set invalid values to a negative value
                }
            }
            if (cnt_valid == 0) {
                ROS_WARN("No valid depth in the depth image");
                m_depth_image_.reset();
                return false;
            }
            scan.array() *= m_setting_.depth_scale;  // convert to meters
        }
        m_depth_image_.reset();
        return true;
    }

    void
    TryUpdate(const ros::Time &time) {
        if (!m_lidar_scan_2d_ && !m_lidar_scan_3d_ && !m_depth_image_) {
            ROS_WARN("No scan data available");
            return;
        }
        const auto [ok, rotation, translation] = GetSensorPose(time);
        if (!ok) {
            ROS_WARN("Failed to get sensor pose");
            return;
        }

        bool are_points = false;
        MatrixX scan;
        switch (m_setting_.scan_type) {
            case ScanType::Laser:
                if (!GetScanFromLaserScan(scan)) { return; }
                are_points = false;
                break;
            case ScanType::PointCloud:
                if (!GetScanFromPointCloud2(scan)) { return; }
                are_points = true;
                break;
            case ScanType::Depth:
                if (!GetScanFromDepthImage(scan)) { return; }
                are_points = false;
                break;
        }
        const bool in_local = m_setting_.scan_in_local_frame;
        if (!are_points && m_setting_.convert_scan_to_points) {
            if (Dim == 2) {
                const std::vector<Vector2> &ray_directions =
                    m_scan_frame_2d_->GetRayDirectionsInFrame();
                MatrixX scan_points(2, scan.size());
                if (in_local) {  // convert to points in local frame
                    for (long i = 0; i < scan.size(); ++i) {
                        scan_points.col(i) = scan.data()[i] * ray_directions[i];
                    }
                } else {
                    Matrix2 rotation_2d = rotation.template block<2, 2>(0, 0);
                    Vector2 translation_2d = translation.template head<2>();
                    for (long i = 0; i < scan.size(); ++i) {
                        scan_points.col(i) =
                            rotation_2d * (scan.data()[i] * ray_directions[i]) + translation_2d;
                    }
                }
                scan = scan_points;
            } else {
                m_scan_frame_3d_->UpdateRanges(MatrixX(rotation), VectorX(translation), scan);
                if (in_local) {
                    scan = Eigen::Map<const Matrix3X>(
                        m_scan_frame_3d_->GetHitPointsFrame().data()->data(),
                        3,
                        m_scan_frame_3d_->GetNumHitRays());
                } else {
                    scan = Eigen::Map<const Matrix3X>(
                        m_scan_frame_3d_->GetHitPointsWorld().data()->data(),
                        3,
                        m_scan_frame_3d_->GetNumHitRays());
                }
            }
            are_points = true;
        }
        auto t1 = ros::WallTime::now();
        double surf_mapping_time;
        bool success;
        {
            ERL_BLOCK_TIMER_MSG_TIME("Surface mapping update", surf_mapping_time);
            success = m_surface_mapping_->Update(rotation, translation, scan, are_points, in_local);
        }
        {
            double time_budget_us = 1e6 / m_sdf_mapping_cfg_->update_hz;  // us
            ERL_BLOCK_TIMER_MSG("Update SDF GPs");
            if (ok) { m_sdf_mapping_->UpdateGpSdf(time_budget_us - surf_mapping_time * 1000); }
        }
        // bool success = m_sdf_mapping_->Update(rotation, translation, scan,
        // are_points, in_local);
        auto t2 = ros::WallTime::now();
        m_msg_update_time_.data = (t2 - t1).toSec();
        m_pub_update_time_.publish(m_msg_update_time_);
        ROS_INFO("Update fps: %f", 1.0 / m_msg_update_time_.data);
        if (!success) { ROS_WARN("Failed to update SDF mapping"); }
    }

    // --- service handler: runs Test() on the current map ---
    bool
    CallbackSdfQuery(
        erl_gp_sdf_msgs::SdfQuery::Request &req,
        erl_gp_sdf_msgs::SdfQuery::Response &res) {

        if (!m_sdf_mapping_) {
            ROS_WARN("SDF mapping is not initialized");
            return false;
        }

        using QuerySetting = typename GpSdfMapping::Setting::TestQuery;

        const auto n = static_cast<int>(req.query_points.size());
        Eigen::Map<const Eigen::MatrixXd> positions_org(
            reinterpret_cast<const double *>(req.query_points.data()),
            3,
            n);
        MatrixDX positions = positions_org.topRows<Dim>().template cast<Dtype>();
        VectorX distances(n);
        MatrixDX gradients(Dim, n);
        Variances variances(Dim + 1, n);
        Covariances covariances(Dim * (Dim + 1) / 2, n);
        distances.setConstant(0.0);
        gradients.setConstant(0.0);
        variances.setConstant(1.0e6);

        auto t1 = ros::WallTime::now();
        bool ok = m_sdf_mapping_->Test(positions, distances, gradients, variances, covariances);
        auto t2 = ros::WallTime::now();
        m_msg_query_time_.data = (t2 - t1).toSec();
        ROS_INFO("SDF query took %f seconds", m_msg_query_time_.data);
        m_pub_query_time_.publish(m_msg_query_time_);
        res.success = ok;
        if (!ok) {
            ROS_WARN("Failed to process SDF query");
            return false;
        }

        // convert to double for the response
        Eigen::VectorXd distances_d;
        Eigen::MatrixXd gradients_d;
        Eigen::MatrixXd variances_d;
        Eigen::MatrixXd covariances_d;
        const double *distances_ptr = nullptr;
        const double *gradients_ptr = nullptr;
        const double *variances_ptr = nullptr;
        const double *covariances_ptr = nullptr;
        if (std::is_same_v<Dtype, float>) {
            distances_d = distances.template cast<double>();
            gradients_d = gradients.template cast<double>();
            variances_d = variances.template cast<double>();
            covariances_d = covariances.template cast<double>();
            distances_ptr = distances_d.data();
            gradients_ptr = gradients_d.data();
            variances_ptr = variances_d.data();
            covariances_ptr = covariances_d.data();
        } else {
            distances_ptr = reinterpret_cast<const double *>(distances.data());
            gradients_ptr = reinterpret_cast<const double *>(gradients.data());
            variances_ptr = reinterpret_cast<const double *>(variances.data());
            covariances_ptr = reinterpret_cast<const double *>(covariances.data());
        }

        // store the results in the response
        res.dim = Dim;
        /// SDF
        res.signed_distances.resize(n);
        std::memcpy(
            reinterpret_cast<char *>(res.signed_distances.data()),
            reinterpret_cast<const char *>(distances_ptr),
            n * sizeof(double));
        /// store the remaining results based on the query setting
        const QuerySetting &query_setting = m_sdf_mapping_cfg_->test_query;
        res.compute_gradient = query_setting.compute_gradient;
        res.compute_gradient_variance = query_setting.compute_gradient_variance;
        res.compute_covariance = query_setting.compute_covariance;
        /// gradients
        res.gradients.clear();
        if (query_setting.compute_gradient) {
            res.gradients.resize(n);
            if (Dim == 2) {
                for (int i = 0; i < n; ++i) {
                    res.gradients[i].x = gradients(0, i);
                    res.gradients[i].y = gradients(1, i);
                    res.gradients[i].z = 0.0;  // z is not used in 2D
                }
            } else {
                std::memcpy(
                    reinterpret_cast<char *>(res.gradients.data()),
                    reinterpret_cast<const char *>(gradients_ptr),
                    3 * n * sizeof(double));
            }
        }
        /// variances
        if (query_setting.compute_gradient_variance) {
            res.variances.resize(n * (Dim + 1));
            std::memcpy(
                reinterpret_cast<char *>(res.variances.data()),
                reinterpret_cast<const char *>(variances_ptr),
                n * (Dim + 1) * sizeof(double));
        } else {
            res.variances.resize(n);
            for (int i = 0; i < n; ++i) { res.variances[i] = variances(0, i); }
        }
        /// covariances
        res.covariances.clear();
        if (query_setting.compute_covariance) {
            res.covariances.resize(n * Dim * (Dim + 1) / 2);
            std::memcpy(
                reinterpret_cast<char *>(res.covariances.data()),
                reinterpret_cast<const char *>(covariances_ptr),
                n * Dim * (Dim + 1) / 2 * sizeof(double));
        }
        return true;
    }

    bool
    CallbackLoadMap(
        erl_gp_sdf_msgs::SaveMap::Request &req,
        erl_gp_sdf_msgs::SaveMap::Response &res) {

        if (!m_sdf_mapping_) {
            ROS_WARN("SDF mapping is not initialized");
            res.success = false;
            return false;
        }
        if (req.name.empty()) {
            ROS_WARN("Map file name is empty");
            res.success = false;
            return false;
        }
        std::filesystem::path map_file = req.name;
        map_file = std::filesystem::absolute(map_file);
        if (!std::filesystem::exists(map_file)) {
            ROS_WARN("Map file %s does not exist", map_file.string().c_str());
            res.success = false;
            return false;
        }
        {
            auto lock = m_sdf_mapping_->GetLockGuard();
            using Serializer = serialization::Serialization<GpSdfMapping>;
            res.success = Serializer::Read(map_file, m_sdf_mapping_.get());
        }
        return true;
    }

    bool
    CallbackSaveMap(
        erl_gp_sdf_msgs::SaveMap::Request &req,
        erl_gp_sdf_msgs::SaveMap::Response &res) {
        if (!m_sdf_mapping_) {
            ROS_WARN("SDF mapping is not initialized");
            res.success = false;
            return false;
        }
        if (req.name.empty()) {
            ROS_WARN("Map file name is empty");
            res.success = false;
            return false;
        }
        std::filesystem::path map_file = req.name;
        map_file = std::filesystem::absolute(map_file);
        std::filesystem::create_directories(map_file.parent_path());
        {
            auto lock = m_sdf_mapping_->GetLockGuard();
            using Serializer = serialization::Serialization<GpSdfMapping>;
            res.success = Serializer::Write(map_file, m_sdf_mapping_.get());
        }
        return true;
    }

    bool
    CallbackSaveMesh(
        erl_gp_sdf_msgs::SaveMap::Request &req,
        erl_gp_sdf_msgs::SaveMap::Response &res) {
        if (!m_sdf_mapping_) {
            ROS_WARN("SDF mapping is not initialized");
            res.success = false;
            return false;
        }
        if (req.name.empty()) {
            ROS_WARN("Mesh file name is empty");
            res.success = false;
            return false;
        }
        std::filesystem::path mesh_file = req.name;
        mesh_file = std::filesystem::absolute(mesh_file);
        std::filesystem::create_directories(mesh_file.parent_path());
        std::vector<VectorD> vertices;
        std::vector<Eigen::Vector<int, Dim>> faces;
        {
            auto lock = m_surface_mapping_->GetLockGuard();
            try {
                res.success = m_surface_mapping_->GetMesh(false, vertices, faces);
            } catch (const std::exception &e) {
                ROS_WARN("Failed to get mesh: %s", e.what());
                res.success = false;
                return false;
            }
        }

        if (Dim == 2) {
            open3d::geometry::LineSet line_set;
            line_set.points_.resize(vertices.size());
            for (size_t i = 0; i < vertices.size(); ++i) {
                line_set.points_[i] = Eigen::Vector3d(
                    static_cast<double>(vertices[i](0)),
                    static_cast<double>(vertices[i](1)),
                    0.0);
            }
            line_set.lines_.resize(faces.size());
            for (size_t i = 0; i < faces.size(); ++i) {
                line_set.lines_[i] =
                    Eigen::Vector2i(static_cast<int>(faces[i](0)), static_cast<int>(faces[i](1)));
            }
            res.success &= open3d::io::WriteLineSetToPLY(req.name, line_set);
        } else {
            open3d::geometry::TriangleMesh mesh;
            mesh.vertices_.resize(vertices.size());
            for (size_t i = 0; i < vertices.size(); ++i) {
                mesh.vertices_[i] = Eigen::Vector3d(
                    static_cast<double>(vertices[i](0)),
                    static_cast<double>(vertices[i](1)),
                    static_cast<double>(vertices[i](2)));
            }
            mesh.triangles_.resize(faces.size());
            for (size_t i = 0; i < faces.size(); ++i) {
                mesh.triangles_[i] = Eigen::Vector3i(
                    static_cast<int>(faces[i](0)),
                    static_cast<int>(faces[i](1)),
                    static_cast<int>(faces[i](2)));
            }
            constexpr bool write_ascii = false;
            constexpr bool compressed = false;
            constexpr bool write_vertex_normals = true;
            constexpr bool write_vertex_colors = false;
            constexpr bool write_triangle_uvs = false;
            constexpr bool print_progress = false;
            res.success &= open3d::io::WriteTriangleMeshToPLY(
                req.name,
                mesh,
                write_ascii,
                compressed,
                write_vertex_normals,
                write_vertex_colors,
                write_triangle_uvs,
                print_progress);
        }
        return true;
    }

    void
    CallbackPublishTree(const ros::TimerEvent & /* event */) {
        if (!m_tree_) { return; }
        if (m_pub_tree_.getNumSubscribers() == 0) { return; }  // no subscribers
        {
            auto lock = m_surface_mapping_->GetLockGuard();
            erl::geometry::SaveToOccupancyTreeMsg<Dtype>(
                m_tree_,
                m_setting_.publish_tree_binary,
                m_msg_tree_);
        }
        m_msg_tree_.header.stamp = ros::Time::now();
        m_pub_tree_.publish(m_msg_tree_);
    }

    void
    CallbackPublishSurfacePoints(const ros::TimerEvent & /* event */) {
        if (m_pub_surface_points_.getNumSubscribers() == 0) { return; }  // no subscribers

        using namespace erl::gp_sdf;

        const auto &surf_data_manager = m_surface_mapping_->GetSurfaceDataManager();

        auto &msg = m_msg_surface_points_;
        msg.header.stamp = ros::Time::now();
        ++msg.header.seq;
        for (int i = 0; i < 8; ++i) {
            msg.fields[i].count = static_cast<uint32_t>(surf_data_manager.Size());
        }
        msg.width = static_cast<uint32_t>(surf_data_manager.Size());
        msg.row_step = msg.point_step * msg.width;
        msg.data.resize(msg.row_step * msg.height);
        float *ptr = reinterpret_cast<float *>(msg.data.data());
        for (auto d: surf_data_manager) {
            ptr[0] = static_cast<float>(d.position.x());
            ptr[1] = static_cast<float>(d.position.y());
            ptr[2] = Dim == 3 ? static_cast<float>(d.position[2]) : 0.0f;
            ptr[3] = static_cast<float>(d.normal.x());
            ptr[4] = static_cast<float>(d.normal.y());
            ptr[5] = Dim == 3 ? static_cast<float>(d.normal[2]) : 0.0f;
            ptr[6] = static_cast<float>(d.var_position);
            ptr[7] = static_cast<float>(d.var_normal);
            ptr += 8;
        }
        m_pub_surface_points_.publish(msg);
    }
};

template<typename Dtype, int Dim>
int
Spin(ros::NodeHandle &nh) {
    SdfMappingNode<Dtype, Dim> node(nh);
    // ros::spin();
    ros::AsyncSpinner spinner(0);  // use all available threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}

int
main(int argc, char **argv) {
    ros::init(argc, argv, "sdf_mapping_node");
    ros::NodeHandle nh("~");  // ~: shorthand for the private namespace

    int map_dim = 3;
    // ROS_ASSERT_MSG is disabled when ROS_ASSERT_ENABLED is false or not defined
    ERL_ASSERTM(nh.param<int>("map_dim", map_dim, 3), "Failed to get map_dim parameter");
    ERL_ASSERTM(map_dim == 2 || map_dim == 3, "map_dim must be either 2 or 3");

    bool double_precision = false;
    ERL_ASSERTM(
        nh.param<bool>("double_precision", double_precision, false),
        "Failed to get double_precision parameter");

    if (double_precision) {
        if (map_dim == 2) { return Spin<double, 2>(nh); }
        return Spin<double, 3>(nh);
    }

    if (map_dim == 2) { return Spin<float, 2>(nh); }
    return Spin<float, 3>(nh);
}
