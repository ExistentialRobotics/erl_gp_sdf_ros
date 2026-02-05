#include "erl_common/block_timer.hpp"
#include "erl_common/eigen.hpp"
#include "erl_common/enum_parse.hpp"
#include "erl_common/ros2_topic_params.hpp"
#include "erl_common/yaml.hpp"
#include "erl_geometry/abstract_occupancy_octree.hpp"
#include "erl_geometry/abstract_occupancy_quadtree.hpp"
#include "erl_geometry/depth_frame_3d.hpp"
#include "erl_geometry/lidar_frame_2d.hpp"
#include "erl_geometry/lidar_frame_3d.hpp"
#include "erl_geometry_msgs/ros2/occupancy_tree_msg.hpp"
#include "erl_gp_sdf/bayesian_hilbert_surface_mapping.hpp"
#include "erl_gp_sdf/gp_occ_surface_mapping.hpp"
#include "erl_gp_sdf/gp_sdf_mapping.hpp"
#include "erl_gp_sdf_msgs/srv/save_map.hpp"
#include "erl_gp_sdf_msgs/srv/sdf_query.hpp"

#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_default_plugins/displays/pointcloud/point_cloud_helpers.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace erl::common;
using namespace erl::common::ros_params;

static rclcpp::Node *g_curr_node = nullptr;

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
    // erl::gp_sdf::GpOccSurfaceMapping<float, 2>, you should use its setting class
    // erl::gp_sdf::GpOccSurfaceMapping<float, 2>::Setting.
    std::string surface_mapping_setting_type = "";
    // path to the yaml file for the surface mapping setting
    std::string surface_mapping_setting_file = "";
    // surface mapping class type. For example, erl::gp_sdf::GpOccSurfaceMapping<float, 2>.
    std::string surface_mapping_type = "";
    // path to the yaml file for the SDF mapping setting
    std::string sdf_mapping_setting_file = "";
    // whether to use the odometry topic to get the sensor pose
    bool use_odom = false;
    // parameters of the odometry topic
    Ros2TopicParams odom_topic{"/jackal_velocity_controller/odom"};
    // can be "odometry" or "transform_stamped"
    OdomType odom_msg_type = OdomType::Odometry;
    // size of the odometry queue
    int odom_queue_size = 100;
    // name of the world frame
    std::string world_frame = "map";
    // name of the sensor frame
    std::string sensor_frame = "front_laser";
    // parameters of the scan topic
    Ros2TopicParams scan_topic{"/front/scan"};
    // can be "laser", "point_cloud", or "depth"
    ScanType scan_type = ScanType::Laser;
    // frame class of the scan. e.g. erl::geometry::LidarFrame3D<float>,
    // erl::geometry::DepthFrame3D<float> for 3D scans. For 2D scans, the only option is
    // erl::geometry::LidarFrame2D<float> or erl::geometry::LidarFrame2D<double>.
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
    // if true, use binary format to publish the occupancy tree, which makes the message smaller.
    bool publish_tree_binary = true;
    // frequency to publish the occupancy tree
    double publish_tree_frequency = 5.0;
    // parameters of the topic to publish the occupancy tree
    Ros2TopicParams tree_topic{"/surface_mapping_tree"};
    // if true, publish the surface points used by the sdf mapping.
    bool publish_surface_points = false;
    // frequency to publish the surface points
    double publish_surface_points_frequency = 5.0;
    // parameters of the topic to publish the surface points
    Ros2TopicParams surface_points_topic{"surface_points"};
    // parameters of the topic to publish the update time
    Ros2TopicParams update_time_topic{"update_time"};
    // parameters of the topic to publish the query time
    Ros2TopicParams query_time_topic{"query_time"};
    // parameters of the service to predict SDF values
    Ros2TopicParams sdf_query_service{"sdf_query", "services"};
    // parameters of the service to save the map
    Ros2TopicParams save_map_service{"save_map", "services"};
    // parameters of the service to load the map
    Ros2TopicParams load_map_service{"load_map", "services"};

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
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, load_map_service));

    bool
    PostDeserialization() override {
        auto logger = g_curr_node->get_logger();

        // RCLCPP_INFO(logger, "Loaded SdfMappingNodeConfig:\n%s", AsYamlString().c_str());

        if (surface_mapping_setting_type.empty()) {
            RCLCPP_WARN(logger, "You must set ~surface_mapping_setting_type");
            return false;
        }
        if (surface_mapping_setting_file.empty()) {
            RCLCPP_WARN(logger, "You must set ~surface_mapping_config");
            return false;
        }
        if (!std::filesystem::exists(surface_mapping_setting_file)) {
            RCLCPP_WARN(
                logger,
                "Surface mapping setting file %s does not exist",
                surface_mapping_setting_file.c_str());
            return false;
        }
        if (surface_mapping_type.empty()) {
            RCLCPP_WARN(logger, "You must set surface_mapping_type");
            return false;
        }
        if (sdf_mapping_setting_file.empty()) {
            RCLCPP_WARN(logger, "You must set sdf_mapping_setting_file");
            return false;
        }
        if (!std::filesystem::exists(sdf_mapping_setting_file)) {
            RCLCPP_WARN(
                logger,
                "SDF mapping setting file %s does not exist",
                sdf_mapping_setting_file.c_str());
            return false;
        }
        if (use_odom && odom_topic.path.empty()) {
            RCLCPP_WARN(logger, "odom_topic.path is empty but use_odom is true");
            return false;
        }
        if (odom_queue_size <= 0) {
            RCLCPP_WARN(logger, "odom_queue_size must be positive");
            return false;
        }
        if (world_frame.empty()) {
            RCLCPP_WARN(logger, "world_frame is empty");
            return false;
        }
        if (!use_odom && sensor_frame.empty()) {
            RCLCPP_WARN(logger, "sensor_frame is empty but use_odom is false");
            return false;
        }
        if (scan_topic.path.empty()) {
            RCLCPP_WARN(logger, "scan_topic is empty");
            return false;
        }
        if (scan_stride <= 0) {
            RCLCPP_WARN(logger, "scan_stride must be positive");
            return false;
        }
        if (convert_scan_to_points) {
            if (scan_frame_setting_file.empty()) {
                RCLCPP_WARN(logger, "For scan conversion, scan_frame_setting_file must be set.");
                return false;
            }
            if (!std::filesystem::exists(scan_frame_setting_file)) {
                RCLCPP_WARN(
                    logger,
                    "Scan frame setting file %s does not exist.",
                    scan_frame_setting_file.c_str());
                return false;
            }
        }
        if (publish_tree && tree_topic.path.empty()) {
            RCLCPP_WARN(logger, "tree_topic.path is empty but publish_tree is true");
            return false;
        }
        if (publish_tree && publish_tree_frequency <= 0.0) {
            RCLCPP_WARN(logger, "publish_tree_frequency must be positive");
            return false;
        }
        if (publish_surface_points && surface_points_topic.path.empty()) {
            RCLCPP_WARN(
                logger,
                "surface_points_topic.path is empty but publish_surface_points is true");
            return false;
        }
        if (publish_surface_points && publish_surface_points_frequency <= 0.0) {
            RCLCPP_WARN(logger, "publish_surface_points_frequency must be positive");
            return false;
        }
        if (update_time_topic.path.empty()) {
            RCLCPP_WARN(logger, "update_time_topic.path is empty");
            return false;
        }
        if (query_time_topic.path.empty()) {
            RCLCPP_WARN(logger, "query_time_topic.path is empty");
            return false;
        }
        if (sdf_query_service.path.empty()) {
            RCLCPP_WARN(logger, "sdf_query_service.path is empty");
            return false;
        }
        if (save_map_service.path.empty()) {
            RCLCPP_WARN(logger, "save_map_service.path is empty");
            return false;
        }
        if (load_map_service.path.empty()) {
            RCLCPP_WARN(logger, "load_map_service.path is empty");
            return false;
        }
        return true;
    }
};

template<typename Dtype, int Dim>
class SdfMappingNode : public rclcpp::Node {
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
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr m_sub_odom_transform_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_sub_laser_scan_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_sub_point_cloud_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_sub_depth_image_;
    rclcpp::Service<erl_gp_sdf_msgs::srv::SdfQuery>::SharedPtr m_srv_query_sdf_;
    rclcpp::Service<erl_gp_sdf_msgs::srv::SaveMap>::SharedPtr m_srv_load_map_;
    rclcpp::Service<erl_gp_sdf_msgs::srv::SaveMap>::SharedPtr m_srv_save_map_;
    rclcpp::Publisher<erl_geometry_msgs::msg::OccupancyTreeMsg>::SharedPtr m_pub_tree_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pub_surface_points_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_pub_update_time_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_pub_query_time_;
    rclcpp::TimerBase::SharedPtr m_pub_tree_timer_;
    rclcpp::TimerBase::SharedPtr m_pub_surface_points_timer_;
    erl_geometry_msgs::msg::OccupancyTreeMsg m_msg_tree_;
    sensor_msgs::msg::PointCloud2 m_msg_surface_points_;
    std_msgs::msg::Float64 m_msg_update_time_;
    std_msgs::msg::Float64 m_msg_query_time_;

    std::shared_ptr<YamlableBase> m_surface_mapping_cfg_ = nullptr;
    std::shared_ptr<AbstractSurfaceMapping> m_surface_mapping_ = nullptr;
    std::shared_ptr<typename GpSdfMapping::Setting> m_sdf_mapping_cfg_ = nullptr;
    std::shared_ptr<GpSdfMapping> m_sdf_mapping_ = nullptr;
    std::shared_ptr<const Tree> m_tree_ = nullptr;  // used to store the occupancy tree

    // for the sensor pose

    std::mutex m_odom_queue_lock_;
    std::vector<geometry_msgs::msg::TransformStamped> m_odom_queue_{};
    int m_odom_queue_head_ = -1;
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener_;

    // for the scan data

    sensor_msgs::msg::LaserScan::ConstSharedPtr m_lidar_scan_2d_ = nullptr;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr m_lidar_scan_3d_ = nullptr;
    sensor_msgs::msg::Image::ConstSharedPtr m_depth_image_ = nullptr;

    std::shared_ptr<LidarFrame2D> m_scan_frame_2d_ = nullptr;
    std::shared_ptr<RangeSensorFrame3D> m_scan_frame_3d_ = nullptr;

public:
    SdfMappingNode()
        : rclcpp::Node("sdf_mapping_node"),
          m_tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
          m_tf_listener_(std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_)) {

        g_curr_node = this;

        auto logger = this->get_logger();
        if (!LoadParameters()) {
            RCLCPP_FATAL(logger, "Failed to load parameters");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(logger, "Loaded node parameters:\n%s", m_setting_.AsYamlString().c_str());

        auto &setting_factory = YamlableBase::Factory::GetInstance();

        // load the surface mapping config
        m_surface_mapping_cfg_ = setting_factory.Create(m_setting_.surface_mapping_setting_type);
        if (!m_surface_mapping_cfg_) {
            RCLCPP_FATAL(logger, "Failed to create surface mapping config");
            rclcpp::shutdown();
            return;
        }
        try {
            if (!m_surface_mapping_cfg_->FromYamlFile(m_setting_.surface_mapping_setting_file)) {
                RCLCPP_FATAL(
                    logger,
                    "Failed to load %s with surface mapping type %s",
                    m_setting_.surface_mapping_setting_file.c_str(),
                    m_setting_.surface_mapping_type.c_str());
                rclcpp::shutdown();
                return;
            }
        } catch (const std::exception &e) {
            RCLCPP_FATAL(
                logger,
                "Failed to load %s with surface mapping type %s: %s",
                m_setting_.surface_mapping_setting_file.c_str(),
                m_setting_.surface_mapping_type.c_str(),
                e.what());
            rclcpp::shutdown();
            return;
        }

        // load the sdf mapping config
        m_sdf_mapping_cfg_ = std::make_shared<typename GpSdfMapping::Setting>();
        try {
            if (!m_sdf_mapping_cfg_->FromYamlFile(m_setting_.sdf_mapping_setting_file)) {
                RCLCPP_FATAL(
                    logger,
                    "Failed to load %s",
                    m_setting_.sdf_mapping_setting_file.c_str());
                rclcpp::shutdown();
                return;
            }
        } catch (const std::exception &e) {
            RCLCPP_FATAL(
                logger,
                "Failed to load %s with SDF mapping type %s: %s",
                m_setting_.sdf_mapping_setting_file.c_str(),
                m_setting_.surface_mapping_type.c_str(),
                e.what());
            rclcpp::shutdown();
            return;
        }

        // create the surface mapping
        m_surface_mapping_ =
            AbstractSurfaceMapping::Create(m_setting_.surface_mapping_type, m_surface_mapping_cfg_);
        m_sdf_mapping_ = std::make_shared<GpSdfMapping>(m_sdf_mapping_cfg_, m_surface_mapping_);
        RCLCPP_INFO(
            logger,
            "Created surface mapping of type %s",
            m_setting_.surface_mapping_type.c_str());
        RCLCPP_INFO(
            logger,
            "Surface mapping config:\n%s",
            m_surface_mapping_cfg_->AsYamlString().c_str());
        RCLCPP_INFO(logger, "SDF mapping config:\n%s", m_sdf_mapping_cfg_->AsYamlString().c_str());

        if (m_setting_.use_odom) {
            switch (m_setting_.odom_msg_type) {
                case OdomType::Odometry: {
                    m_sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
                        m_setting_.odom_topic.path,
                        m_setting_.odom_topic.GetQoS(),
                        std::bind(
                            &SdfMappingNode::CallbackOdomOdometry,
                            this,
                            std::placeholders::_1));
                    break;
                }
                case OdomType::TransformStamped: {
                    m_sub_odom_transform_ =
                        this->create_subscription<geometry_msgs::msg::TransformStamped>(
                            m_setting_.odom_topic.path,
                            m_setting_.odom_topic.GetQoS(),
                            std::bind(
                                &SdfMappingNode::CallbackOdomTransformStamped,
                                this,
                                std::placeholders::_1));
                    break;
                }
                default: {
                    RCLCPP_FATAL(
                        logger,
                        "Invalid odometry message type: %d",
                        static_cast<int>(m_setting_.odom_msg_type));
                    rclcpp::shutdown();
                    return;
                }
            }
            m_odom_queue_.reserve(m_setting_.odom_queue_size);
        }

        switch (m_setting_.scan_type) {
            case ScanType::Laser:
                RCLCPP_INFO(
                    logger,
                    "Subscribing to %s as laser scan",
                    m_setting_.scan_topic.path.c_str());
                m_sub_laser_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                    m_setting_.scan_topic.path,
                    m_setting_.scan_topic.GetQoS(),
                    std::bind(&SdfMappingNode::CallbackLaserScan, this, std::placeholders::_1));
                break;
            case ScanType::PointCloud:
                RCLCPP_INFO(
                    logger,
                    "Subscribing to %s as point cloud",
                    m_setting_.scan_topic.path.c_str());
                m_sub_point_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    m_setting_.scan_topic.path,
                    m_setting_.scan_topic.GetQoS(),
                    std::bind(&SdfMappingNode::CallbackPointCloud2, this, std::placeholders::_1));
                break;
            case ScanType::Depth:
                RCLCPP_INFO(
                    logger,
                    "Subscribing to %s as depth image",
                    m_setting_.scan_topic.path.c_str());
                m_sub_depth_image_ = this->create_subscription<sensor_msgs::msg::Image>(
                    m_setting_.scan_topic.path,
                    m_setting_.scan_topic.GetQoS(),
                    std::bind(&SdfMappingNode::CallbackDepthImage, this, std::placeholders::_1));
                break;
        }

        if (m_setting_.convert_scan_to_points) {
            if (Dim == 2) {
                auto frame_setting = std::make_shared<typename LidarFrame2D::Setting>();
                try {
                    if (!frame_setting->FromYamlFile(m_setting_.scan_frame_setting_file)) {
                        RCLCPP_FATAL(
                            logger,
                            "Failed to load %s with frame type %s",
                            m_setting_.scan_frame_setting_file.c_str(),
                            m_setting_.surface_mapping_type.c_str());
                        rclcpp::shutdown();
                        return;
                    }
                } catch (const std::exception &e) {
                    RCLCPP_FATAL(
                        logger,
                        "Failed to load %s with frame type %s: %s",
                        m_setting_.scan_frame_setting_file.c_str(),
                        m_setting_.surface_mapping_type.c_str(),
                        e.what());
                    rclcpp::shutdown();
                    return;
                }
                if (m_setting_.scan_stride > 1) {
                    frame_setting->Resize(1.0f / static_cast<Dtype>(m_setting_.scan_stride));
                }
                m_scan_frame_2d_ = std::make_shared<LidarFrame2D>(frame_setting);
                RCLCPP_INFO(
                    logger,
                    "Created scan frame of type %s with setting:\n%s",
                    m_setting_.scan_frame_type.c_str(),
                    frame_setting->AsYamlString().c_str());
            } else if (m_setting_.scan_frame_type == type_name<LidarFrame3D>()) {
                auto frame_setting = std::make_shared<typename LidarFrame3D::Setting>();
                try {
                    if (!frame_setting->FromYamlFile(m_setting_.scan_frame_setting_file)) {
                        RCLCPP_FATAL(
                            logger,
                            "Failed to load %s",
                            m_setting_.scan_frame_setting_file.c_str());
                        rclcpp::shutdown();
                        return;
                    }
                } catch (const std::exception &e) {
                    RCLCPP_FATAL(
                        logger,
                        "Failed to load %s with frame type %s: %s",
                        m_setting_.scan_frame_setting_file.c_str(),
                        m_setting_.surface_mapping_type.c_str(),
                        e.what());
                    rclcpp::shutdown();
                    return;
                }
                if (m_setting_.scan_stride > 1) {
                    frame_setting->Resize(1.0f / static_cast<Dtype>(m_setting_.scan_stride));
                }
                m_scan_frame_3d_ = std::make_shared<LidarFrame3D>(frame_setting);
                RCLCPP_INFO(
                    logger,
                    "Created scan frame of type %s with setting:\n%s",
                    m_setting_.scan_frame_type.c_str(),
                    frame_setting->AsYamlString().c_str());
            } else if (m_setting_.scan_frame_type == type_name<DepthFrame3D>()) {
                auto frame_setting = std::make_shared<typename DepthFrame3D::Setting>();
                try {
                    if (!frame_setting->FromYamlFile(m_setting_.scan_frame_setting_file)) {
                        RCLCPP_FATAL(
                            logger,
                            "Failed to load %s",
                            m_setting_.scan_frame_setting_file.c_str());
                        rclcpp::shutdown();
                        return;
                    }
                } catch (const std::exception &e) {
                    RCLCPP_FATAL(
                        logger,
                        "Failed to load %s with frame type %s: %s",
                        m_setting_.scan_frame_setting_file.c_str(),
                        m_setting_.surface_mapping_type.c_str(),
                        e.what());
                    rclcpp::shutdown();
                    return;
                }
                if (m_setting_.scan_stride > 1) {
                    frame_setting->Resize(1.0f / static_cast<Dtype>(m_setting_.scan_stride));
                }
                m_scan_frame_3d_ = std::make_shared<DepthFrame3D>(frame_setting);
                RCLCPP_INFO(
                    logger,
                    "Created scan frame of type %s with setting:\n%s",
                    m_setting_.scan_frame_type.c_str(),
                    frame_setting->AsYamlString().c_str());
            } else {
                RCLCPP_FATAL(
                    logger,
                    "Invalid scan frame type: %s",
                    m_setting_.scan_frame_type.c_str());
                rclcpp::shutdown();
                return;
            }
        }

// advertise the service to query the SDF mapping
#ifdef ROS_HUMBLE
        m_srv_query_sdf_ = this->create_service<erl_gp_sdf_msgs::srv::SdfQuery>(
            m_setting_.sdf_query_service.path,
            std::bind(
                &SdfMappingNode::CallbackSdfQuery,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            m_setting_.sdf_query_service.GetQoS().get_rmw_qos_profile());
        m_srv_load_map_ = this->create_service<erl_gp_sdf_msgs::srv::SaveMap>(
            m_setting_.load_map_service.path,
            std::bind(
                &SdfMappingNode::CallbackLoadMap,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            m_setting_.load_map_service.GetQoS().get_rmw_qos_profile());
        m_srv_save_map_ = this->create_service<erl_gp_sdf_msgs::srv::SaveMap>(
            m_setting_.save_map_service.path,
            std::bind(
                &SdfMappingNode::CallbackSaveMap,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            m_setting_.save_map_service.GetQoS().get_rmw_qos_profile());
#else
        m_srv_query_sdf_ = this->create_service<erl_gp_sdf_msgs::srv::SdfQuery>(
            m_setting_.sdf_query_service.path,
            std::bind(
                &SdfMappingNode::CallbackSdfQuery,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            m_setting_.sdf_query_service.GetQoS());
        m_srv_load_map_ = this->create_service<erl_gp_sdf_msgs::srv::SaveMap>(
            m_setting_.load_map_service.path,
            std::bind(
                &SdfMappingNode::CallbackLoadMap,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            m_setting_.load_map_service.GetQoS());
        m_srv_save_map_ = this->create_service<erl_gp_sdf_msgs::srv::SaveMap>(
            m_setting_.save_map_service.path,
            std::bind(
                &SdfMappingNode::CallbackSaveMap,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            m_setting_.save_map_service.GetQoS());
#endif
        // publish the occupancy tree used by the surface mapping
        if (m_setting_.publish_tree) {
            if (!TryToGetSurfaceMappingTree()) {
                RCLCPP_FATAL(logger, "Failed to get surface mapping tree");
                rclcpp::shutdown();
                return;
            }
            m_pub_tree_ = this->create_publisher<erl_geometry_msgs::msg::OccupancyTreeMsg>(
                m_setting_.tree_topic.path,
                m_setting_.tree_topic.GetQoS());
            m_pub_tree_timer_ = this->create_wall_timer(
                std::chrono::duration<double>(1.0 / m_setting_.publish_tree_frequency),
                std::bind(&SdfMappingNode::CallbackPublishTree, this));
            m_msg_tree_.header.frame_id = m_setting_.world_frame;
            m_msg_tree_.header.stamp = this->get_clock()->now();
        }

        // publish the surface points used by the sdf mapping
        if (m_setting_.publish_surface_points) {
            m_pub_surface_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                m_setting_.surface_points_topic.path,
                m_setting_.surface_points_topic.GetQoS());
            m_pub_surface_points_timer_ = this->create_wall_timer(
                std::chrono::duration<double>(1.0 / m_setting_.publish_surface_points_frequency),
                std::bind(&SdfMappingNode::CallbackPublishSurfacePoints, this));
            m_msg_surface_points_.header.frame_id = m_setting_.world_frame;
            m_msg_surface_points_.header.stamp = this->get_clock()->now();
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
                m_msg_surface_points_.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
            }
            m_msg_surface_points_.point_step = 32;       // 8 fields * 4 bytes each
            m_msg_surface_points_.is_bigendian = false;  // little-endian
            m_msg_surface_points_.is_dense = false;      // there may be NaN values in the normals
            m_msg_surface_points_.height = 1;            // unorganized point cloud
        }

        m_pub_update_time_ = this->create_publisher<std_msgs::msg::Float64>(
            m_setting_.update_time_topic.path,
            m_setting_.update_time_topic.GetQoS());
        m_pub_query_time_ = this->create_publisher<std_msgs::msg::Float64>(
            m_setting_.query_time_topic.path,
            m_setting_.query_time_topic.GetQoS());
        m_msg_update_time_.data = 0.0;
        m_msg_query_time_.data = 0.0;

        RCLCPP_INFO(logger, "SdfMappingNode is ready. Waiting for scans + queries...");
    }

private:
    bool
    LoadParameters() {
        if (!m_setting_.LoadFromRos2(this, "")) { return false; }

        auto logger = this->get_logger();

        // more checks
        if (m_setting_.convert_scan_to_points) {
            if (Dim == 2) {
                if (m_setting_.scan_frame_type != type_name<LidarFrame2D>()) {
                    RCLCPP_WARN(
                        logger,
                        "For 2D scans, scan_frame_type is %s but must be %s.",
                        m_setting_.scan_frame_type.c_str(),
                        type_name<LidarFrame2D>().c_str());
                }
            } else {
                if (m_setting_.scan_frame_type != type_name<LidarFrame3D>() &&
                    m_setting_.scan_frame_type != type_name<DepthFrame3D>()) {
                    RCLCPP_WARN(
                        logger,
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
    GetSensorPose(const rclcpp::Time &time) {
        auto logger = this->get_logger();
        if (m_setting_.use_odom) {
            geometry_msgs::msg::TransformStamped transform;

            {
                std::lock_guard<std::mutex> lock(m_odom_queue_lock_);
                // get the latest odometry message
                const int &head = m_odom_queue_head_;
                if (head < 0) {
                    RCLCPP_WARN(logger, "No odometry message available");
                    return {false, Rotation::Identity(), Translation::Zero()};
                }
                geometry_msgs::msg::TransformStamped *transform_ptr = nullptr;
                for (int i = head; i >= 0; --i) {
                    if (m_odom_queue_[i].header.stamp.nanosec <= time.nanoseconds()) {
                        transform_ptr = &m_odom_queue_[i];
                        break;
                    }
                }
                if (!transform_ptr) {  // search older messages
                    const int size = static_cast<int>(m_odom_queue_.size());
                    for (int i = size - 1; i > head; --i) {
                        if (m_odom_queue_[i].header.stamp.nanosec <= time.nanoseconds()) {
                            transform_ptr = &m_odom_queue_[i];
                            break;
                        }
                    }
                }
                if (!transform_ptr) {
                    RCLCPP_WARN(logger, "No odometry msg available for time %f", time.seconds());
                    return {false, Rotation::Identity(), Translation::Zero()};
                }
                transform = *transform_ptr;  // copy the transform
            }

            if (transform.child_frame_id != m_setting_.sensor_frame) {
                geometry_msgs::msg::TransformStamped tf_child_to_sensor;
                try {
                    tf_child_to_sensor = m_tf_buffer_->lookupTransform(
                        transform.child_frame_id,
                        m_setting_.sensor_frame,
                        transform.header.stamp,
                        rclcpp::Duration::from_seconds(0.5));
                } catch (tf2::LookupException &ex) {
                    RCLCPP_WARN(
                        logger,
                        "Failed to lookup transform from %s to %s: %s",
                        transform.child_frame_id.c_str(),
                        m_setting_.sensor_frame.c_str(),
                        ex.what());
                    return {false, Rotation::Identity(), Translation::Zero()};
                }
                tf2::doTransform(tf_child_to_sensor, transform, transform);
            }

            auto &pose = transform.transform;
            // erase the dimension of rotation and translation temporarily
            MatrixX rotation;
            VectorX translation;
            if (Dim == 2) {
                tf2::Quaternion q(
                    pose.rotation.x,
                    pose.rotation.y,
                    pose.rotation.z,
                    pose.rotation.w);
                tf2::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);
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
        if (m_setting_.world_frame == m_setting_.sensor_frame) {
            // identity transform
            return {true, Rotation::Identity(), Translation::Zero()};
        }
        // get the latest transform from the tf buffer
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped = m_tf_buffer_->lookupTransform(
                m_setting_.world_frame,
                m_setting_.sensor_frame,
                time,
                rclcpp::Duration::from_seconds(5.0));
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(logger, "%s", ex.what());
            return {false, Rotation::Identity(), Translation::Zero()};
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
    CallbackOdomOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
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
            geometry_msgs::msg::TransformStamped transform;
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
    CallbackOdomTransformStamped(const geometry_msgs::msg::TransformStamped::SharedPtr msg) {
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
    CallbackLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        m_lidar_scan_2d_ = msg;
        TryUpdate(msg->header.stamp);
    }

    void
    CallbackPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        m_lidar_scan_3d_ = msg;
        TryUpdate(msg->header.stamp);
    }

    void
    CallbackDepthImage(const sensor_msgs::msg::Image::SharedPtr msg) {
        m_depth_image_ = msg;
        TryUpdate(msg->header.stamp);
    }

    bool
    GetScanFromLaserScan(MatrixX &scan) {
        if (!m_lidar_scan_2d_) {
            RCLCPP_WARN(this->get_logger(), "No laser scan data available");
            return false;
        }
        auto &scan_msg = *m_lidar_scan_2d_;
        if (scan_msg.ranges.empty()) {
            RCLCPP_WARN(this->get_logger(), "Laser scan data is empty");
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
        auto logger = this->get_logger();
        if (!m_lidar_scan_3d_) {
            RCLCPP_WARN(logger, "No point cloud data available");
            return false;
        }
        auto &cloud = *m_lidar_scan_3d_;
        if (cloud.fields.empty() || cloud.data.empty()) {
            RCLCPP_WARN(logger, "Point cloud data is empty");
            m_lidar_scan_3d_.reset();
            return false;
        }
        if (cloud.data.size() != cloud.width * cloud.height * cloud.point_step) {
            RCLCPP_WARN(logger, "Point cloud data size doesn't match width, height, or point step");
            m_lidar_scan_3d_.reset();
            return false;
        }
        if (cloud.row_step != cloud.width * cloud.point_step) {
            RCLCPP_WARN(logger, "Point cloud row step does not match width and point step");
            m_lidar_scan_3d_.reset();
            return false;
        }

        // validate x, y, z are present
        const int32_t xi = rviz_default_plugins::findChannelIndex(m_lidar_scan_3d_, "x");
        const int32_t yi = rviz_default_plugins::findChannelIndex(m_lidar_scan_3d_, "y");
        const int32_t zi = rviz_default_plugins::findChannelIndex(m_lidar_scan_3d_, "z");
        if (xi < 0 || yi < 0 || zi < 0) {
            RCLCPP_WARN(logger, "Point cloud does not contain x, y, z fields");
            m_lidar_scan_3d_.reset();
            return false;
        }
        // validate x, y, z fields have the same data type
        const uint8_t &xtype = cloud.fields[xi].datatype;
        const uint8_t &ytype = cloud.fields[yi].datatype;
        const uint8_t &ztype = cloud.fields[zi].datatype;
        if (xtype != ytype || xtype != ztype || ytype != ztype) {
            RCLCPP_WARN(logger, "Point cloud x, y, z fields have different data types");
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
        if (xtype == sensor_msgs::msg::PointField::FLOAT32) {
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
        } else if (xtype == sensor_msgs::msg::PointField::FLOAT64) {
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
            RCLCPP_WARN(logger, "Unsupported point cloud data type %d", xtype);
            m_lidar_scan_3d_.reset();
            return false;
        }
        if (point_count == 0) {
            RCLCPP_WARN(logger, "No valid points in point cloud");
            m_lidar_scan_3d_.reset();
            return false;
        }
        scan.conservativeResize(3, point_count);
        m_lidar_scan_3d_.reset();
        return true;
    }

    bool
    GetScanFromDepthImage(MatrixX &scan) {
        auto logger = this->get_logger();
        if (!m_depth_image_) {
            RCLCPP_WARN(logger, "No depth image available");
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
            RCLCPP_WARN(logger, "Unsupported depth encoding %s", m_depth_image_->encoding.c_str());
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
                RCLCPP_WARN(logger, "No valid depth in the depth image");
                m_depth_image_.reset();
                return false;
            }
            scan.array() *= m_setting_.depth_scale;  // convert to meters
        }
        m_depth_image_.reset();
        return true;
    }

    void
    TryUpdate(const rclcpp::Time &time) {
        if (!m_lidar_scan_2d_ && !m_lidar_scan_3d_ && !m_depth_image_) {
            RCLCPP_WARN(this->get_logger(), "No scan data available");
            return;
        }
        const auto [ok, rotation, translation] = GetSensorPose(time);
        if (!ok) {
            RCLCPP_WARN(this->get_logger(), "Failed to get sensor pose");
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
        auto t1 = this->get_clock()->now();
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
        // bool success = m_sdf_mapping_->Update(rotation, translation, scan, are_points, in_local);
        auto t2 = this->get_clock()->now();
        m_msg_update_time_.data = (t2 - t1).seconds();
        m_pub_update_time_->publish(m_msg_update_time_);
        RCLCPP_INFO(this->get_logger(), "Update fps: %f", 1.0 / m_msg_update_time_.data);
        if (!success) { RCLCPP_WARN(this->get_logger(), "Failed to update SDF mapping"); }
    }

    // --- service handler: runs Test() on the current map ---
    void
    CallbackSdfQuery(
        const std::shared_ptr<erl_gp_sdf_msgs::srv::SdfQuery::Request> req,
        std::shared_ptr<erl_gp_sdf_msgs::srv::SdfQuery::Response> res) {

        if (!m_sdf_mapping_) {
            RCLCPP_WARN(this->get_logger(), "SDF mapping is not initialized");
            return;
        }

        using QuerySetting = typename GpSdfMapping::Setting::TestQuery;

        const auto n = static_cast<int>(req->query_points.size());
        Eigen::Map<const Eigen::Matrix3Xd> positions_org(
            reinterpret_cast<const double *>(req->query_points.data()),
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

        auto t1 = this->get_clock()->now();
        bool ok = m_sdf_mapping_->Test(positions, distances, gradients, variances, covariances);
        auto t2 = this->get_clock()->now();
        m_msg_query_time_.data = (t2 - t1).seconds();
        RCLCPP_INFO(this->get_logger(), "SDF query took %f seconds", m_msg_query_time_.data);
        m_pub_query_time_->publish(m_msg_query_time_);
        res->success = ok;
        if (!ok) {
            RCLCPP_WARN(this->get_logger(), "Failed to process SDF query");
            return;
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
        res->dim = Dim;
        /// SDF
        res->signed_distances.resize(n);
        std::memcpy(
            reinterpret_cast<char *>(res->signed_distances.data()),
            reinterpret_cast<const char *>(distances_ptr),
            n * sizeof(double));
        /// store the remaining results based on the query setting
        const QuerySetting &query_setting = m_sdf_mapping_cfg_->test_query;
        res->compute_gradient = query_setting.compute_gradient;
        res->compute_gradient_variance = query_setting.compute_gradient_variance;
        res->compute_covariance = query_setting.compute_covariance;
        /// gradients
        res->gradients.clear();
        if (query_setting.compute_gradient) {
            res->gradients.resize(n);
            if (Dim == 2) {
                for (int i = 0; i < n; ++i) {
                    res->gradients[i].x = gradients(0, i);
                    res->gradients[i].y = gradients(1, i);
                    res->gradients[i].z = 0.0;  // z is not used in 2D
                }
            } else {
                std::memcpy(
                    reinterpret_cast<char *>(res->gradients.data()),
                    reinterpret_cast<const char *>(gradients_ptr),
                    3 * n * sizeof(double));
            }
        }
        /// variances
        if (query_setting.compute_gradient_variance) {
            res->variances.resize(n * (Dim + 1));
            std::memcpy(
                reinterpret_cast<char *>(res->variances.data()),
                reinterpret_cast<const char *>(variances_ptr),
                n * (Dim + 1) * sizeof(double));
        } else {
            res->variances.resize(n);
            for (int i = 0; i < n; ++i) { res->variances[i] = variances(0, i); }
        }
        /// covariances
        res->covariances.clear();
        if (query_setting.compute_covariance) {
            res->covariances.resize(n * Dim * (Dim + 1) / 2);
            std::memcpy(
                reinterpret_cast<char *>(res->covariances.data()),
                reinterpret_cast<const char *>(covariances_ptr),
                n * Dim * (Dim + 1) / 2 * sizeof(double));
        }
    }

    void
    CallbackLoadMap(
        erl_gp_sdf_msgs::srv::SaveMap::Request::ConstSharedPtr req,
        erl_gp_sdf_msgs::srv::SaveMap::Response::SharedPtr res) {
        auto logger = this->get_logger();
        if (!m_sdf_mapping_) {
            RCLCPP_WARN(logger, "SDF mapping is not initialized");
            res->success = false;
            return;
        }
        if (req->name.empty()) {
            RCLCPP_WARN(logger, "Map file name is empty");
            res->success = false;
            return;
        }
        std::filesystem::path map_file = req->name;
        map_file = std::filesystem::absolute(map_file);
        if (!std::filesystem::exists(map_file)) {
            RCLCPP_WARN(logger, "Map file %s does not exist", map_file.string().c_str());
            res->success = false;
            return;
        }
        {
            auto lock = m_sdf_mapping_->GetLockGuard();
            using Serializer = serialization::Serialization<GpSdfMapping>;
            res->success = Serializer::Read(map_file, m_sdf_mapping_.get());
        }
    }

    void
    CallbackSaveMap(
        erl_gp_sdf_msgs::srv::SaveMap::Request::ConstSharedPtr req,
        erl_gp_sdf_msgs::srv::SaveMap::Response::SharedPtr res) {
        if (!m_sdf_mapping_) {
            RCLCPP_WARN(this->get_logger(), "SDF mapping is not initialized");
            res->success = false;
            return;
        }
        if (req->name.empty()) {
            RCLCPP_WARN(this->get_logger(), "Map file name is empty");
            res->success = false;
            return;
        }
        std::filesystem::path map_file = req->name;
        map_file = std::filesystem::absolute(map_file);
        std::filesystem::create_directories(map_file.parent_path());
        {
            auto lock = m_sdf_mapping_->GetLockGuard();
            using Serializer = serialization::Serialization<GpSdfMapping>;
            res->success = Serializer::Write(map_file, m_sdf_mapping_.get());
        }
    }

    void
    CallbackPublishTree() {
        if (!m_tree_) { return; }
        if (m_pub_tree_->get_subscription_count() == 0) { return; }  // no subscribers
        {
            auto lock = m_surface_mapping_->GetLockGuard();
            erl::geometry::SaveToOccupancyTreeMsg<Dtype>(
                m_tree_,
                m_setting_.publish_tree_binary,
                m_msg_tree_);
        }
        m_msg_tree_.header.stamp = this->get_clock()->now();
        m_pub_tree_->publish(m_msg_tree_);
    }

    void
    CallbackPublishSurfacePoints() {
        if (m_pub_surface_points_->get_subscription_count() == 0) { return; }  // no subscribers

        using namespace erl::gp_sdf;

        const auto &surf_data_manager = m_surface_mapping_->GetSurfaceDataManager();

        auto &msg = m_msg_surface_points_;
        msg.header.stamp = this->get_clock()->now();
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
        m_pub_surface_points_->publish(msg);
    }
};

int
main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Create a node to read parameters
    auto temp_node = rclcpp::Node::make_shared("sdf_mapping_node");

    int map_dim = 3;
    temp_node->declare_parameter<int>("map_dim", 3);
    map_dim = temp_node->get_parameter("map_dim").as_int();
    if (map_dim != 2 && map_dim != 3) {
        RCLCPP_FATAL(temp_node->get_logger(), "map_dim must be either 2 or 3");
        return 1;
    }

    bool double_precision = false;
    temp_node->declare_parameter<bool>("double_precision", false);
    double_precision = temp_node->get_parameter("double_precision").as_bool();
    temp_node.reset();  // release the temporary node

    std::shared_ptr<rclcpp::Node> node;
    if (double_precision) {
        if (map_dim == 2) {
            node = std::make_shared<SdfMappingNode<double, 2>>();
        } else {
            node = std::make_shared<SdfMappingNode<double, 3>>();
        }
    } else {
        if (map_dim == 2) {
            node = std::make_shared<SdfMappingNode<float, 2>>();
        } else {
            node = std::make_shared<SdfMappingNode<float, 3>>();
        }
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
