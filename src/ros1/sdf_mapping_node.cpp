#include "erl_common/block_timer.hpp"
#include "erl_common/eigen.hpp"
#include "erl_common/yaml.hpp"
#include "erl_geometry/abstract_occupancy_octree.hpp"
#include "erl_geometry/abstract_occupancy_quadtree.hpp"
#include "erl_geometry/camera_intrinsic.hpp"
#include "erl_geometry/depth_frame_3d.hpp"
#include "erl_geometry/lidar_frame_2d.hpp"
#include "erl_geometry/lidar_frame_3d.hpp"
#include "erl_geometry_msgs/MeshMsg.h"
#include "erl_geometry_msgs/ros1/occupancy_tree_msg.hpp"
#include "erl_gp_sdf/bayesian_hilbert_surface_mapping.hpp"
#include "erl_gp_sdf/gp_occ_surface_mapping.hpp"
#include "erl_gp_sdf/gp_sdf_mapping.hpp"
#include "erl_gp_sdf/height_map_projector.hpp"
#include "erl_gp_sdf_msgs/OccQuery.h"
#include "erl_gp_sdf_msgs/SaveMap.h"
#include "erl_gp_sdf_msgs/SdfQuery.h"

#include <geometry_msgs/Vector3.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <open3d/geometry/LineSet.h>
#include <open3d/geometry/TriangleMesh.h>
#include <open3d/io/LineSetIO.h>
#include <open3d/io/TriangleMeshIO.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
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

struct OdomParams : Yamlable<OdomParams> {
    // whether to use the odometry topic to get the sensor pose
    bool enabled = false;
    // name of the odometry topic
    std::string topic = "/jackal_velocity_controller/odom";
    // can be "odometry" or "transform_stamped"
    OdomType msg_type = OdomType::Odometry;
    // size of the odometry queue
    int queue_size = 100;

    ERL_REFLECT_SCHEMA(
        OdomParams,
        ERL_REFLECT_MEMBER(OdomParams, enabled),
        ERL_REFLECT_MEMBER(OdomParams, topic),
        ERL_REFLECT_MEMBER(OdomParams, msg_type),
        ERL_REFLECT_MEMBER(OdomParams, queue_size));
};

struct ScanParams : Yamlable<ScanParams> {
    // name of the scan topic
    std::string topic = "/front/scan";
    // can be "laser", "point_cloud", or "depth"
    ScanType type = ScanType::Laser;
    // frame class of the scan. e.g. erl::geometry::LidarFrame3D<float>,
    // erl::geometry::DepthFrame3D<float> for 3D scans. For 2D scans, the only option is
    // erl::geometry::LidarFrame2D<float> or erl::geometry::LidarFrame2D<double>.
    std::string frame_type = "";
    // path to the yaml file for the scan frame setting
    std::string frame_setting_file = "";
    // if stride > 1, the scan will be downsampled by this factor.
    int stride = 1;
    // if true, convert the scan to points when the scan is not a point cloud.
    bool convert_to_points = false;
    // if the scan data is in the local frame, set this to true.
    bool in_local_frame = false;
    // scale for depth image, 0.001 converts mm to m.
    double depth_scale = 0.001;

    ERL_REFLECT_SCHEMA(
        ScanParams,
        ERL_REFLECT_MEMBER(ScanParams, topic),
        ERL_REFLECT_MEMBER(ScanParams, type),
        ERL_REFLECT_MEMBER(ScanParams, frame_type),
        ERL_REFLECT_MEMBER(ScanParams, frame_setting_file),
        ERL_REFLECT_MEMBER(ScanParams, stride),
        ERL_REFLECT_MEMBER(ScanParams, convert_to_points),
        ERL_REFLECT_MEMBER(ScanParams, in_local_frame),
        ERL_REFLECT_MEMBER(ScanParams, depth_scale));
};

struct PaintSurfaceParams : Yamlable<PaintSurfaceParams> {
    // if true, extract per-point colors from the scan point cloud and fold them into surface
    // voxels after each Update. Only effective with BayesianHilbertSurfaceMapping and point cloud
    // input.
    bool color_from_scan = false;
    // if the path is non-empty, subscribe to this topic to paint the surface voxels.
    // The message type depends on topic_type: "pcd" expects PointCloud2 with rgb/rgba fields;
    // "image" expects sensor_msgs/Image (BGR8/RGB8/BGRA8/RGBA8).
    std::string topic = "";
    // "pcd" or "image". Selects PointCloud2- or Image-based voxel painting for the topic
    // subscriber.
    std::string topic_type = "image";
    // if true, PaintVoxels overwrites voxel color. If false, it averages via running mean.
    bool overwrite = true;
    // maximum distance (meters) from sensor for paint points. Only used when topic_type is "pcd".
    // If <= 0, no filtering.
    double max_distance = 0.0;
    // camera intrinsics for the paint image (only used when topic_type is "image").
    erl::geometry::CameraIntrinsic<double> camera_intrinsic;

    ERL_REFLECT_SCHEMA(
        PaintSurfaceParams,
        ERL_REFLECT_MEMBER(PaintSurfaceParams, color_from_scan),
        ERL_REFLECT_MEMBER(PaintSurfaceParams, topic),
        ERL_REFLECT_MEMBER(PaintSurfaceParams, topic_type),
        ERL_REFLECT_MEMBER(PaintSurfaceParams, overwrite),
        ERL_REFLECT_MEMBER(PaintSurfaceParams, max_distance),
        ERL_REFLECT_MEMBER(PaintSurfaceParams, camera_intrinsic));
};

struct PublishTreeParams : Yamlable<PublishTreeParams> {
    // if true, publish the occupancy tree used by the surface mapping.
    bool enabled = false;
    // if true, use binary format to publish the occupancy tree, which makes the message smaller.
    bool binary = true;
    // frequency to publish the occupancy tree
    double frequency = 5.0;
    // the topic to publish the occupancy tree
    std::string topic = "surface_mapping_tree";

    ERL_REFLECT_SCHEMA(
        PublishTreeParams,
        ERL_REFLECT_MEMBER(PublishTreeParams, enabled),
        ERL_REFLECT_MEMBER(PublishTreeParams, binary),
        ERL_REFLECT_MEMBER(PublishTreeParams, frequency),
        ERL_REFLECT_MEMBER(PublishTreeParams, topic));
};

struct PublishSurfacePointsParams : Yamlable<PublishSurfacePointsParams> {
    // if true, publish the surface points used by the sdf mapping.
    bool enabled = false;
    // frequency to publish the surface points
    double frequency = 5.0;
    // the topic to publish the surface points
    std::string topic = "surface_points";

    ERL_REFLECT_SCHEMA(
        PublishSurfacePointsParams,
        ERL_REFLECT_MEMBER(PublishSurfacePointsParams, enabled),
        ERL_REFLECT_MEMBER(PublishSurfacePointsParams, frequency),
        ERL_REFLECT_MEMBER(PublishSurfacePointsParams, topic));
};

struct PublishMeshParams : Yamlable<PublishMeshParams> {
    // if true, publish the mesh built by the surface mapping.
    bool enabled = false;
    // frequency to publish the mesh
    double frequency = 1.0;
    // the topic to publish the mesh
    std::string topic = "mesh";

    ERL_REFLECT_SCHEMA(
        PublishMeshParams,
        ERL_REFLECT_MEMBER(PublishMeshParams, enabled),
        ERL_REFLECT_MEMBER(PublishMeshParams, frequency),
        ERL_REFLECT_MEMBER(PublishMeshParams, topic));
};

struct PublishOccupancyGridParams : Yamlable<PublishOccupancyGridParams> {
    // if true, publish the 2D occupancy grid projected from the 3D height map (Dim==3 only).
    bool enabled = false;
    // frequency to publish the occupancy grid
    double frequency = 5.0;
    // the topic to publish the occupancy grid
    std::string topic = "occupancy_grid";
    // height map projector setting (required when enabled is true)
    erl::gp_sdf::HeightMapProjectorSetting<double> height_map_projector;

    ERL_REFLECT_SCHEMA(
        PublishOccupancyGridParams,
        ERL_REFLECT_MEMBER(PublishOccupancyGridParams, enabled),
        ERL_REFLECT_MEMBER(PublishOccupancyGridParams, frequency),
        ERL_REFLECT_MEMBER(PublishOccupancyGridParams, topic),
        ERL_REFLECT_MEMBER(PublishOccupancyGridParams, height_map_projector));
};

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
    // grouped parameters for odometry input.
    OdomParams odom;
    // name of the world frame
    std::string world_frame = "map";
    // name of the sensor frame
    std::string sensor_frame = "front_laser";
    // grouped parameters for the scan input.
    ScanParams scan;
    // grouped parameters for surface painting.
    PaintSurfaceParams paint_surface;
    // grouped parameters for publishing the occupancy tree.
    PublishTreeParams publish_tree;
    // grouped parameters for publishing the surface points.
    PublishSurfacePointsParams publish_surface_points;
    // grouped parameters for publishing the mesh.
    PublishMeshParams publish_mesh;
    // grouped parameters for publishing the 2D occupancy grid.
    PublishOccupancyGridParams publish_occupancy_grid;
    // the topic to publish the update time
    std::string update_time_topic = "update_time";
    // the topic to publish the query time
    std::string query_time_topic = "query_time";
    // the service to query the SDF value
    std::string sdf_query_service = "sdf_query";
    // the service to query occupancy values
    std::string occ_query_service = "occ_query";
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
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, odom),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, world_frame),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, sensor_frame),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, scan),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, paint_surface),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, publish_tree),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, publish_surface_points),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, publish_mesh),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, publish_occupancy_grid),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, update_time_topic),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, query_time_topic),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, sdf_query_service),
        ERL_REFLECT_MEMBER(SdfMappingNodeConfig, occ_query_service),
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
        if (odom.enabled && odom.topic.empty()) {
            ROS_WARN("odom.topic is empty but odom.enabled is true");
            return false;
        }
        if (odom.queue_size <= 0) {
            ROS_WARN("odom.queue_size must be positive");
            return false;
        }
        if (world_frame.empty()) {
            ROS_WARN("World frame is empty");
            return false;
        }
        if (!odom.enabled && sensor_frame.empty()) {
            ROS_WARN("sensor_frame is empty but odom.enabled is false");
            return false;
        }
        if (scan.topic.empty()) {
            ROS_WARN("scan.topic is empty");
            return false;
        }
        if (scan.stride <= 0) {
            ROS_WARN("scan.stride must be positive");
            return false;
        }
        if (scan.type != ScanType::PointCloud && scan.convert_to_points) {
            if (scan.frame_setting_file.empty()) {
                ROS_WARN("For scan conversion, scan.frame_setting_file must be set.");
                return false;
            }
            if (!std::filesystem::exists(scan.frame_setting_file)) {
                ROS_WARN(
                    "Scan frame setting file %s does not exist.",
                    scan.frame_setting_file.c_str());
                return false;
            }
        }
        if (publish_tree.enabled) {
            if (publish_tree.topic.empty()) {
                ROS_WARN("publish_tree.topic is empty but publish_tree.enabled is true");
                return false;
            }
            if (publish_tree.frequency <= 0.0) {
                ROS_WARN("publish_tree.frequency must be positive");
                return false;
            }
        }
        if (publish_surface_points.enabled) {
            if (publish_surface_points.topic.empty()) {
                ROS_WARN(
                    "publish_surface_points.topic is empty but "
                    "publish_surface_points.enabled is true");
                return false;
            }
            if (publish_surface_points.frequency <= 0.0) {
                ROS_WARN("publish_surface_points.frequency must be positive");
                return false;
            }
        }
        if (publish_mesh.enabled) {
            if (publish_mesh.topic.empty()) {
                ROS_WARN("publish_mesh.topic is empty but publish_mesh.enabled is true");
                return false;
            }
            if (publish_mesh.frequency <= 0.0) {
                ROS_WARN("publish_mesh.frequency must be positive");
                return false;
            }
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
        if (occ_query_service.empty()) {
            ROS_WARN("occ_query_service is empty");
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
        if (publish_occupancy_grid.enabled) {
            if (publish_occupancy_grid.topic.empty()) {
                ROS_WARN(
                    "publish_occupancy_grid.topic is empty but "
                    "publish_occupancy_grid.enabled is true");
                return false;
            }
            if (publish_occupancy_grid.frequency <= 0.0) {
                ROS_WARN("publish_occupancy_grid.frequency must be positive");
                return false;
            }
        }
        if (paint_surface.color_from_scan && scan.type != ScanType::PointCloud) {
            ROS_WARN(
                "paint_surface.color_from_scan is true but scan.type is not 'point_cloud'. "
                "Color extraction is only supported for point cloud input. Disabling it.");
            paint_surface.color_from_scan = false;
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
        erl::geometry::AbstractOccupancyQuadtree<Dtype>,
        erl::geometry::AbstractOccupancyOctree<Dtype>>;
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
    using ColorMatrix = Eigen::Matrix<uint8_t, 4, Eigen::Dynamic>;
    using Color = typename AbstractSurfaceMapping::Color;
    using RangeSensorFrame3D = erl::geometry::RangeSensorFrame3D<Dtype>;
    using LidarFrame2D = erl::geometry::LidarFrame2D<Dtype>;
    using LidarFrame3D = erl::geometry::LidarFrame3D<Dtype>;
    using DepthFrame3D = erl::geometry::DepthFrame3D<Dtype>;

    SdfMappingNodeConfig m_setting_;
    ros::NodeHandle m_nh_;
    ros::Subscriber m_sub_odom_;
    ros::Subscriber m_sub_scan_;
    ros::Subscriber m_sub_paint_surface_;
    ros::Subscriber m_sub_paint_surface_image_;
    Matrix3 m_paint_intrinsic_matrix_ = Matrix3::Zero();
    cv::Mat m_paint_image_bgra_buffer_;
    bool m_paint_surface_enabled_ = false;
    ros::ServiceServer m_srv_query_sdf_;
    ros::ServiceServer m_srv_occ_query_;
    ros::ServiceServer m_srv_load_map_;
    ros::ServiceServer m_srv_save_map_;
    ros::ServiceServer m_srv_save_mesh_;
    ros::Publisher m_pub_tree_;
    ros::Publisher m_pub_surface_points_;
    ros::Publisher m_pub_mesh_;
    ros::Publisher m_pub_update_time_;
    ros::Publisher m_pub_query_time_;
    ros::Publisher m_pub_occupancy_grid_;
    ros::Timer m_pub_tree_timer_;
    ros::Timer m_pub_surface_points_timer_;
    ros::Timer m_pub_mesh_timer_;
    ros::Timer m_pub_occupancy_grid_timer_;
    erl_geometry_msgs::OccupancyTreeMsg m_msg_tree_;
    sensor_msgs::PointCloud2 m_msg_surface_points_;
    erl_geometry_msgs::MeshMsg m_msg_mesh_;
    std_msgs::Float64 m_msg_update_time_;
    std_msgs::Float64 m_msg_query_time_;
    nav_msgs::OccupancyGrid m_msg_occupancy_grid_;

    std::shared_ptr<YamlableBase> m_surface_mapping_cfg_ = nullptr;
    std::shared_ptr<AbstractSurfaceMapping> m_surface_mapping_ = nullptr;
    std::shared_ptr<BayesianHilbertSurfaceMapping> m_bhm_ = nullptr;
    std::shared_ptr<typename GpSdfMapping::Setting> m_sdf_mapping_cfg_ = nullptr;
    std::shared_ptr<GpSdfMapping> m_sdf_mapping_ = nullptr;
    std::shared_ptr<const Tree> m_tree_ = nullptr;
    std::unique_ptr<erl::gp_sdf::HeightMapProjector<Dtype>> m_height_map_projector_ = nullptr;

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

    // for color storage
    ColorMatrix m_point_colors_;  // 4xN RGBA, populated when color_from_scan is true
    bool m_received_colors_ = false;

public:
    SdfMappingNode(ros::NodeHandle &nh)
        : m_nh_(nh) {
        if (!LoadParameters()) {
            ROS_FATAL("Failed to load parameters");
            ros::shutdown();
            return;
        }

        ROS_INFO("Loaded node parameters:\n%s", m_setting_.AsYamlString().c_str());

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
        if (!m_surface_mapping_) {
            ROS_FATAL(
                "Failed to create surface mapping of type %s",
                m_setting_.surface_mapping_type.c_str());
            ros::shutdown();
            return;
        }
        m_sdf_mapping_ = std::make_shared<GpSdfMapping>(m_sdf_mapping_cfg_, m_surface_mapping_);
        if (!m_setting_.paint_surface.topic.empty()) {
            if (m_setting_.paint_surface.color_from_scan) {
                ROS_WARN(
                    "paint_surface.topic is set, ignoring color_from_scan. "
                    "Coloring will come from the paint_surface.topic instead.");
                m_setting_.paint_surface.color_from_scan = false;
            }
        }
        m_bhm_ = std::dynamic_pointer_cast<BayesianHilbertSurfaceMapping>(m_surface_mapping_);
        if (m_setting_.paint_surface.color_from_scan) {
            if (m_bhm_) {
                ROS_INFO(
                    "color_from_scan is true; per-point colors will be folded into surface voxels "
                    "after each Update via PaintVoxels.");
            } else {
                ROS_WARN(
                    "color_from_scan is true but surface mapping is not a "
                    "BayesianHilbertSurfaceMapping. Disabling color_from_scan.");
                m_setting_.paint_surface.color_from_scan = false;
            }
        }
        ROS_INFO("Created surface mapping of type %s", m_setting_.surface_mapping_type.c_str());
        ROS_INFO("Surface mapping config:\n%s", m_surface_mapping_cfg_->AsYamlString().c_str());
        ROS_INFO("SDF mapping config:\n%s", m_sdf_mapping_cfg_->AsYamlString().c_str());

        if (m_setting_.odom.enabled) {
            switch (m_setting_.odom.msg_type) {
                case OdomType::Odometry:
                    ROS_INFO(
                        "Subscribing to %s as nav_msgs/Odometry",
                        m_setting_.odom.topic.c_str());
                    m_sub_odom_ = m_nh_.subscribe(
                        m_setting_.odom.topic,
                        1,
                        &SdfMappingNode::CallbackOdomOdometry,
                        this);
                    break;
                case OdomType::TransformStamped:
                    ROS_INFO(
                        "Subscribing to %s as geometry_msgs/TransformStamped",
                        m_setting_.odom.topic.c_str());
                    m_sub_odom_ = m_nh_.subscribe(
                        m_setting_.odom.topic,
                        1,
                        &SdfMappingNode::CallbackOdomTransformStamped,
                        this);
                    break;
                default:
                    ROS_FATAL(
                        "Invalid odometry message type: %d",
                        static_cast<int>(m_setting_.odom.msg_type));
                    ros::shutdown();
                    return;
            }
            m_odom_queue_.reserve(m_setting_.odom.queue_size);
        }

        bool are_points = false;
        switch (m_setting_.scan.type) {
            case ScanType::Laser:
                ROS_INFO("Subscribing to %s as laser scan", m_setting_.scan.topic.c_str());
                m_sub_scan_ = m_nh_.subscribe(
                    m_setting_.scan.topic,
                    10,
                    &SdfMappingNode::CallbackLaserScan,
                    this);
                break;
            case ScanType::PointCloud:
                ROS_INFO("Subscribing to %s as point cloud", m_setting_.scan.topic.c_str());
                are_points = true;
                m_sub_scan_ = m_nh_.subscribe(
                    m_setting_.scan.topic,
                    10,
                    &SdfMappingNode::CallbackPointCloud2,
                    this);
                break;
            case ScanType::Depth:
                ROS_INFO("Subscribing to %s as depth image", m_setting_.scan.topic.c_str());
                m_sub_scan_ = m_nh_.subscribe(
                    m_setting_.scan.topic,
                    10,
                    &SdfMappingNode::CallbackDepthImage,
                    this);
                break;
        }

        if constexpr (Dim == 3) {
            if (!m_setting_.paint_surface.topic.empty()) {
                if (m_bhm_) {
                    if (m_setting_.paint_surface.topic_type == "pcd") {
                        ROS_INFO(
                            "Subscribing to %s (PointCloud2) for voxel painting",
                            m_setting_.paint_surface.topic.c_str());
                        m_sub_paint_surface_ = m_nh_.subscribe(
                            m_setting_.paint_surface.topic,
                            10,
                            &SdfMappingNode::CallbackPaintVoxelsPcd,
                            this);
                    } else {
                        m_paint_intrinsic_matrix_ =
                            m_setting_.paint_surface.camera_intrinsic.GetIntrinsicMatrix()
                                .template cast<Dtype>();
                        ROS_INFO(
                            "Subscribing to %s (Image) for voxel painting with fx=%f fy=%f cx=%f "
                            "cy=%f",
                            m_setting_.paint_surface.topic.c_str(),
                            m_setting_.paint_surface.camera_intrinsic.camera_fx,
                            m_setting_.paint_surface.camera_intrinsic.camera_fy,
                            m_setting_.paint_surface.camera_intrinsic.camera_cx,
                            m_setting_.paint_surface.camera_intrinsic.camera_cy);
                        m_sub_paint_surface_image_ = m_nh_.subscribe(
                            m_setting_.paint_surface.topic,
                            10,
                            &SdfMappingNode::CallbackPaintVoxelsImage,
                            this);
                    }
                    m_paint_surface_enabled_ = true;
                } else {
                    ROS_WARN(
                        "paint_surface.topic is set but surface mapping is not a "
                        "BayesianHilbertSurfaceMapping. Ignoring paint_surface.");
                }
            }
        } else {  // constexpr Dim == 2
            if (!m_setting_.paint_surface.topic.empty()) {
                ROS_WARN("paint_surface.topic is only supported for 3D. Ignoring.");
            }
        }

        if (!are_points && m_setting_.scan.convert_to_points) {
            if (Dim == 2) {
                auto frame_setting = std::make_shared<typename LidarFrame2D::Setting>();
                try {
                    if (!frame_setting->FromYamlFile(m_setting_.scan.frame_setting_file)) {
                        ROS_FATAL(
                            "Failed to load %s with frame type %s",
                            m_setting_.scan.frame_setting_file.c_str(),
                            m_setting_.scan.frame_type.c_str());
                        ros::shutdown();
                        return;
                    }
                } catch (const std::exception &e) {
                    ROS_FATAL(
                        "Failed to parse %s with frame type %s: %s",
                        m_setting_.scan.frame_setting_file.c_str(),
                        m_setting_.scan.frame_type.c_str(),
                        e.what());
                    ros::shutdown();
                    return;
                }
                if (m_setting_.scan.stride > 1) {
                    frame_setting->Resize(1.0f / static_cast<Dtype>(m_setting_.scan.stride));
                }
                m_scan_frame_2d_ = std::make_shared<LidarFrame2D>(frame_setting);
                ROS_INFO(
                    "Created scan frame of type %s with setting:\n%s",
                    m_setting_.scan.frame_type.c_str(),
                    frame_setting->AsYamlString().c_str());
            } else if (m_setting_.scan.frame_type == type_name<LidarFrame3D>()) {
                auto frame_setting = std::make_shared<typename LidarFrame3D::Setting>();
                try {
                    if (!frame_setting->FromYamlFile(m_setting_.scan.frame_setting_file)) {
                        ROS_FATAL(
                            "Failed to load %s with frame type %s",
                            m_setting_.scan.frame_setting_file.c_str(),
                            m_setting_.scan.frame_type.c_str());
                        ros::shutdown();
                        return;
                    }
                } catch (const std::exception &e) {
                    ROS_FATAL(
                        "Failed to parse %s with frame type %s: %s",
                        m_setting_.scan.frame_setting_file.c_str(),
                        m_setting_.scan.frame_type.c_str(),
                        e.what());
                    ros::shutdown();
                    return;
                }
                if (m_setting_.scan.stride > 1) {
                    frame_setting->Resize(1.0f / static_cast<Dtype>(m_setting_.scan.stride));
                }
                m_scan_frame_3d_ = std::make_shared<LidarFrame3D>(frame_setting);
                ROS_INFO(
                    "Created scan frame of type %s with setting:\n%s",
                    m_setting_.scan.frame_type.c_str(),
                    frame_setting->AsYamlString().c_str());
            } else if (m_setting_.scan.frame_type == type_name<DepthFrame3D>()) {
                auto frame_setting = std::make_shared<typename DepthFrame3D::Setting>();
                try {
                    if (!frame_setting->FromYamlFile(m_setting_.scan.frame_setting_file)) {
                        ROS_FATAL(
                            "Failed to load %s with frame type %s",
                            m_setting_.scan.frame_setting_file.c_str(),
                            m_setting_.scan.frame_type.c_str());
                        ros::shutdown();
                        return;
                    }
                } catch (const std::exception &e) {
                    ROS_FATAL(
                        "Failed to parse %s with frame type %s: %s",
                        m_setting_.scan.frame_setting_file.c_str(),
                        m_setting_.scan.frame_type.c_str(),
                        e.what());
                    ros::shutdown();
                    return;
                }
                if (m_setting_.scan.stride > 1) {
                    frame_setting->Resize(1.0f / static_cast<Dtype>(m_setting_.scan.stride));
                }
                m_scan_frame_3d_ = std::make_shared<DepthFrame3D>(frame_setting);
                ROS_INFO(
                    "Created scan frame of type %s with setting:\n%s",
                    m_setting_.scan.frame_type.c_str(),
                    frame_setting->AsYamlString().c_str());
            } else {
                ROS_FATAL("Invalid scan frame type: %s", m_setting_.scan.frame_type.c_str());
                ros::shutdown();
                return;
            }
        }

        // advertise the service to query the SDF mapping
        m_srv_query_sdf_ = m_nh_.advertiseService(
            m_setting_.sdf_query_service,
            &SdfMappingNode::CallbackSdfQuery,
            this);
        m_srv_occ_query_ = m_nh_.advertiseService(
            m_setting_.occ_query_service,
            &SdfMappingNode::CallbackOccQuery,
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
        if (m_setting_.publish_tree.enabled) {
            if (!TryToGetSurfaceMappingTree()) {
                ROS_FATAL("Failed to get surface mapping tree");
                ros::shutdown();
                return;
            }
            m_pub_tree_ = m_nh_.advertise<erl_geometry_msgs::OccupancyTreeMsg>(
                m_setting_.publish_tree.topic,
                1,
                true);
            m_pub_tree_timer_ = m_nh_.createTimer(
                ros::Duration(1.0 / m_setting_.publish_tree.frequency),
                &SdfMappingNode::CallbackPublishTree,
                this);
            m_msg_tree_.header.frame_id = m_setting_.world_frame;
            m_msg_tree_.header.seq = -1;
        }

        // publish the surface points used by the sdf mapping
        if (m_setting_.publish_surface_points.enabled) {
            m_pub_surface_points_ = m_nh_.advertise<sensor_msgs::PointCloud2>(
                m_setting_.publish_surface_points.topic,
                1,
                true);
            m_pub_surface_points_timer_ = m_nh_.createTimer(
                ros::Duration(1.0 / m_setting_.publish_surface_points.frequency),
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

        // publish the mesh built by the surface mapping
        if (m_setting_.publish_mesh.enabled) {
            m_pub_mesh_ =
                m_nh_.advertise<erl_geometry_msgs::MeshMsg>(m_setting_.publish_mesh.topic, 1, true);
            m_msg_mesh_.header.frame_id = m_setting_.world_frame;
            m_msg_mesh_.header.seq = -1;
            m_pub_mesh_timer_ = m_nh_.createTimer(
                ros::Duration(1.0 / m_setting_.publish_mesh.frequency),
                &SdfMappingNode::CallbackPublishMesh,
                this);
        }

        // publish the 2D occupancy grid projected from the 3D height map
        if constexpr (Dim == 3) {
            if (m_setting_.publish_occupancy_grid.enabled) {
                using HMPSetting = erl::gp_sdf::HeightMapProjectorSetting<Dtype>;
                auto hmp_setting = std::make_shared<HMPSetting>();
                const auto &src = m_setting_.publish_occupancy_grid.height_map_projector;
                hmp_setting->target_resolution = static_cast<Dtype>(src.target_resolution);
                hmp_setting->robot_height = static_cast<Dtype>(src.robot_height);
                hmp_setting->max_step_height = static_cast<Dtype>(src.max_step_height);
                hmp_setting->ground_z_min = static_cast<Dtype>(src.ground_z_min);
                hmp_setting->ground_z_max = static_cast<Dtype>(src.ground_z_max);
                hmp_setting->min_normal_z = static_cast<Dtype>(src.min_normal_z);
                hmp_setting->sensor_z_change_threshold =
                    static_cast<Dtype>(src.sensor_z_change_threshold);
                hmp_setting->use_bounding_box = src.use_bounding_box;
                hmp_setting->bounding_box = src.bounding_box.template Cast<Dtype>();

                m_height_map_projector_ =
                    std::make_unique<erl::gp_sdf::HeightMapProjector<Dtype>>(hmp_setting);

                m_pub_occupancy_grid_ = m_nh_.advertise<nav_msgs::OccupancyGrid>(
                    m_setting_.publish_occupancy_grid.topic,
                    1,
                    true);
                m_msg_occupancy_grid_.header.frame_id = m_setting_.world_frame;

                m_pub_occupancy_grid_timer_ = m_nh_.createTimer(
                    ros::Duration(1.0 / m_setting_.publish_occupancy_grid.frequency),
                    &SdfMappingNode::CallbackPublishOccupancyGrid,
                    this);
            }
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
        if (m_setting_.scan.convert_to_points) {
            if (Dim == 2) {
                ROS_WARN_COND(
                    m_setting_.scan.frame_type != type_name<LidarFrame2D>(),
                    "For 2D scans, scan_frame_type is %s but must be %s.",
                    m_setting_.scan.frame_type.c_str(),
                    type_name<LidarFrame2D>().c_str());
            } else {
                if (m_setting_.scan.frame_type != type_name<LidarFrame3D>() &&
                    m_setting_.scan.frame_type != type_name<DepthFrame3D>()) {
                    ROS_WARN(
                        "For 3D scans, scan_frame_type must be %s or %s. Not %s.",
                        type_name<LidarFrame3D>().c_str(),
                        type_name<DepthFrame3D>().c_str(),
                        m_setting_.scan.frame_type.c_str());
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
        if (!m_bhm_) { return nullptr; }
        return m_bhm_->GetTree();
    }

    // get the pose for time t
    std::tuple<bool, Rotation, Translation>
    GetSensorPose(const ros::Time &time) {
        if (m_setting_.odom.enabled) {
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
        if (m_setting_.world_frame == m_setting_.sensor_frame) {
            return {true, Rotation::Identity(), Translation::Zero()};
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
        if (static_cast<int>(m_odom_queue_.size()) >= m_setting_.odom.queue_size) {
            auto &transform = m_odom_queue_[m_odom_queue_head_];
            transform.header = msg->header;
            transform.child_frame_id = msg->child_frame_id;
            transform.transform.rotation = msg->pose.pose.orientation;
            transform.transform.translation.x = msg->pose.pose.position.x;
            transform.transform.translation.y = msg->pose.pose.position.y;
            transform.transform.translation.z = msg->pose.pose.position.z;
            m_odom_queue_head_ = (m_odom_queue_head_ + 1) % m_setting_.odom.queue_size;
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
        if (static_cast<int>(m_odom_queue_.size()) >= m_setting_.odom.queue_size) {
            auto &transform = m_odom_queue_[m_odom_queue_head_];
            transform.header = msg->header;
            transform.child_frame_id = msg->child_frame_id;
            transform.transform = msg->transform;
            m_odom_queue_head_ = (m_odom_queue_head_ + 1) % m_setting_.odom.queue_size;
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

    void
    CallbackPaintVoxelsPcd(const sensor_msgs::PointCloud2::ConstPtr &msg) {
        if constexpr (Dim != 3) {
            (void) msg;
            return;
        } else {  // NOLINT
            if (!m_bhm_) {
                ROS_WARN_ONCE(
                    "PaintVoxels callback: BayesianHilbertSurfaceMapping not available, skipping.");
                return;
            }

            // look up transform from point cloud frame to world frame
            geometry_msgs::TransformStamped tf_stamped;
            try {
                tf_stamped = m_tf_buffer_.lookupTransform(
                    m_setting_.world_frame,
                    msg->header.frame_id,
                    msg->header.stamp,
                    ros::Duration(0.5));
            } catch (tf2::TransformException &ex) {
                ROS_WARN("PaintTree: TF lookup failed: %s", ex.what());
                return;
            }
            const auto &tf = tf_stamped.transform;
            const Matrix3 rotation =
                Eigen::Quaternion<Dtype>(tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z)
                    .toRotationMatrix();
            const Vector3 translation(tf.translation.x, tf.translation.y, tf.translation.z);

            const auto &cloud = *msg;
            if (cloud.fields.empty() || cloud.data.empty()) { return; }
            if (cloud.data.size() != cloud.width * cloud.height * cloud.point_step) { return; }

            // validate x, y, z fields
            const int32_t xi = rviz::findChannelIndex(msg, "x");
            const int32_t yi = rviz::findChannelIndex(msg, "y");
            const int32_t zi = rviz::findChannelIndex(msg, "z");
            if (xi < 0 || yi < 0 || zi < 0) {
                ROS_WARN("PaintTree: point cloud has no x/y/z fields");
                return;
            }
            const uint8_t xtype = cloud.fields[xi].datatype;
            if (cloud.fields[yi].datatype != xtype || cloud.fields[zi].datatype != xtype) {
                ROS_WARN("PaintTree: x/y/z fields have different data types");
                return;
            }
            const uint32_t xoff = cloud.fields[xi].offset;
            const uint32_t yoff = cloud.fields[yi].offset;
            const uint32_t zoff = cloud.fields[zi].offset;

            // look for rgb/rgba field
            int32_t rgb_field_idx = rviz::findChannelIndex(msg, "rgb");
            bool has_alpha = false;
            if (rgb_field_idx < 0) {
                rgb_field_idx = rviz::findChannelIndex(msg, "rgba");
                has_alpha = (rgb_field_idx >= 0);
            }
            uint32_t rgb_off = 0;
            uint8_t rgb_type = 0;
            if (rgb_field_idx >= 0) {
                rgb_off = cloud.fields[rgb_field_idx].offset;
                rgb_type = cloud.fields[rgb_field_idx].datatype;
                if (rgb_type != sensor_msgs::PointField::FLOAT32 &&
                    rgb_type != sensor_msgs::PointField::UINT32) {
                    ROS_WARN_ONCE("PaintTree: unsupported rgb field type %d, skipping.", rgb_type);
                    return;
                }
            } else {
                ROS_WARN_ONCE("PaintTree: point cloud has no rgb/rgba field, skipping.");
                return;
            }

            const auto width = static_cast<int>(cloud.width);
            const auto height = static_cast<int>(cloud.height);
            const uint32_t point_step = cloud.point_step;
            const uint32_t row_step = cloud.row_step;
            const long max_points = static_cast<long>(width) * height;

            // extract points and colors
            Matrix3X points(3, max_points);
            ColorMatrix colors(4, max_points);
            long count = 0;
            const Dtype max_dist_sq = m_setting_.paint_surface.max_distance > 0.0
                                          ? static_cast<Dtype>(
                                                m_setting_.paint_surface.max_distance *
                                                m_setting_.paint_surface.max_distance)
                                          : Dtype(0);

            auto extract_point = [&](const uint8_t *ptr, auto read_coord) {
                Dtype x = static_cast<Dtype>(read_coord(ptr + xoff));
                Dtype y = static_cast<Dtype>(read_coord(ptr + yoff));
                Dtype z = static_cast<Dtype>(read_coord(ptr + zoff));
                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) { return; }
                if (max_dist_sq > 0 && (x * x + y * y + z * z) > max_dist_sq) { return; }
                auto point = points.col(count);
                point[0] = x;
                point[1] = y;
                point[2] = z;
                const auto *bgra = ptr + rgb_off;
                auto color = colors.col(count);
                color[0] = bgra[2];  // R
                color[1] = bgra[1];  // G
                color[2] = bgra[0];  // B
                color[3] = has_alpha ? bgra[3] : static_cast<uint8_t>(255);
                ++count;
            };

            if (xtype == sensor_msgs::PointField::FLOAT32) {
                for (int h = 0; h < height; ++h) {
                    const uint8_t *ptr = cloud.data.data() + h * row_step;
                    for (int w = 0; w < width; ++w) {
                        extract_point(ptr, [](const uint8_t *p) {
                            return *reinterpret_cast<const float *>(p);
                        });
                        ptr += point_step;
                    }
                }
            } else if (xtype == sensor_msgs::PointField::FLOAT64) {
                for (int h = 0; h < height; ++h) {
                    const uint8_t *ptr = cloud.data.data() + h * row_step;
                    for (int w = 0; w < width; ++w) {
                        extract_point(ptr, [](const uint8_t *p) {
                            return *reinterpret_cast<const double *>(p);
                        });
                        ptr += point_step;
                    }
                }
            } else {
                ROS_WARN("PaintTree: unsupported point field type %d", xtype);
                return;
            }

            if (count == 0) { return; }
            points.conservativeResize(3, count);
            colors.conservativeResize(4, count);

            // transform points to world frame
            points = (rotation * points).colwise() + translation;

            (void) m_bhm_->PaintVoxels(
                points,
                colors,
                m_setting_.paint_surface.overwrite,
                /*parallel=*/true);
        }  // else (Dim == 3)
    }

    void
    CallbackPaintVoxelsImage(const sensor_msgs::Image::ConstPtr &msg) {
        if constexpr (Dim != 3) {
            (void) msg;
            return;
        } else {  // NOLINT
            if (!m_bhm_) {
                ROS_WARN_ONCE(
                    "PaintVoxels(image) callback: BayesianHilbertSurfaceMapping not available, "
                    "skipping.");
                return;
            }
            if (msg->data.empty() || msg->width == 0 || msg->height == 0) { return; }

            const auto &enc = msg->encoding;
            int src_cv_type = 0;
            int cvt_code = -1;
            if (enc == sensor_msgs::image_encodings::BGRA8) {
                src_cv_type = CV_8UC4;
            } else if (enc == sensor_msgs::image_encodings::RGBA8) {
                src_cv_type = CV_8UC4;
                cvt_code = cv::COLOR_RGBA2BGRA;
            } else if (enc == sensor_msgs::image_encodings::BGR8) {
                src_cv_type = CV_8UC3;
                cvt_code = cv::COLOR_BGR2BGRA;
            } else if (enc == sensor_msgs::image_encodings::RGB8) {
                src_cv_type = CV_8UC3;
                cvt_code = cv::COLOR_RGB2BGRA;
            } else {
                ROS_WARN_ONCE(
                    "PaintVoxels(image): unsupported encoding '%s' (need bgr8/rgb8/bgra8/rgba8), "
                    "skipping.",
                    enc.c_str());
                return;
            }

            geometry_msgs::TransformStamped tf_stamped;
            try {
                tf_stamped = m_tf_buffer_.lookupTransform(
                    m_setting_.world_frame,
                    msg->header.frame_id,
                    msg->header.stamp,
                    ros::Duration(0.5));
            } catch (tf2::TransformException &ex) {
                ROS_WARN("PaintVoxels(image): TF lookup failed: %s", ex.what());
                return;
            }
            const auto &tf = tf_stamped.transform;
            const Matrix3 rotation_world_cam =
                Eigen::Quaternion<Dtype>(tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z)
                    .toRotationMatrix();
            const Vector3 translation_world_cam(
                tf.translation.x,
                tf.translation.y,
                tf.translation.z);

            cv::Mat src(
                static_cast<int>(msg->height),
                static_cast<int>(msg->width),
                src_cv_type,
                const_cast<uint8_t *>(msg->data.data()),
                static_cast<size_t>(msg->step));
            if (cvt_code < 0) {
                src.copyTo(m_paint_image_bgra_buffer_);
            } else {
                cv::cvtColor(src, m_paint_image_bgra_buffer_, cvt_code);
            }

            (void) m_bhm_->PaintVoxels(
                m_paint_image_bgra_buffer_,
                rotation_world_cam,
                translation_world_cam,
                m_paint_intrinsic_matrix_,
                m_setting_.paint_surface.overwrite,
                /*parallel=*/true);
        }  // else (Dim == 3)
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
        if (m_setting_.scan.stride > 1) {
            scan = DownsampleEigenMatrix(scan, m_setting_.scan.stride, 1);
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

        // color extraction setup
        m_received_colors_ = false;
        bool extract_color = m_setting_.paint_surface.color_from_scan;
        int32_t rgb_field_idx = -1;
        uint32_t rgb_off = 0;
        uint8_t rgb_type = 0;
        bool has_alpha = false;
        if (extract_color) {
            // try "rgb", then "rgba"
            rgb_field_idx = rviz::findChannelIndex(m_lidar_scan_3d_, "rgb");
            if (rgb_field_idx < 0) {
                rgb_field_idx = rviz::findChannelIndex(m_lidar_scan_3d_, "rgba");
                has_alpha = (rgb_field_idx >= 0);
            }
            if (rgb_field_idx >= 0) {
                rgb_off = cloud.fields[rgb_field_idx].offset;
                rgb_type = cloud.fields[rgb_field_idx].datatype;
                if (rgb_type != sensor_msgs::PointField::FLOAT32 &&
                    rgb_type != sensor_msgs::PointField::UINT32) {
                    ROS_WARN_ONCE(
                        "color_from_scan: unsupported rgb field type %d, "
                        "disabling color extraction.",
                        rgb_type);
                    rgb_field_idx = -1;
                }
            }
            if (rgb_field_idx < 0) {
                ROS_WARN_ONCE(
                    "color_from_scan is true but point cloud has no usable rgb/rgba field. "
                    "Skipping color extraction for this message.");
                extract_color = false;
            }
        }

        uint32_t point_step = cloud.point_step;
        uint32_t row_step = cloud.row_step;
        auto width = static_cast<int>(cloud.width);
        auto height = static_cast<int>(cloud.height);
        const int scan_stride = m_setting_.scan.stride;
        if (scan_stride > 1) {
            width = (width + scan_stride - 1) / scan_stride;
            height = (height + scan_stride - 1) / scan_stride;
            point_step *= scan_stride;
            row_step *= scan_stride;
        }
        scan.resize(3, width * height);
        if (extract_color && m_point_colors_.cols() < scan.cols()) {
            m_point_colors_.resize(4, scan.cols());
        }
        long point_count = 0;

        // lambda to extract RGBA from a point's data pointer
        auto extract_rgba = [&](const uint8_t *ptr, long idx) {
            if (!extract_color) { return; }
            // rgb_field_idx and rgb_type are guaranteed valid when extract_color is true
            const auto *bgra = ptr + rgb_off;
            auto point_color = m_point_colors_.col(idx);
            point_color[0] = bgra[2];  // R
            point_color[1] = bgra[1];  // G
            point_color[2] = bgra[0];  // B
            point_color[3] = has_alpha ? bgra[3] : static_cast<uint8_t>(255);
        };

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
                        extract_rgba(ptr, point_count);
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
                        extract_rgba(ptr, point_count);
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
        if (extract_color) {
            m_point_colors_.conservativeResize(4, point_count);
            m_received_colors_ = true;
        }
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
            if (m_setting_.scan.stride > 1) {
                scan = DownsampleEigenMatrix(scan, m_setting_.scan.stride, m_setting_.scan.stride);
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
            scan.array() *= static_cast<Dtype>(m_setting_.scan.depth_scale);  // convert to meters
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
        switch (m_setting_.scan.type) {
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
        const bool in_local = m_setting_.scan.in_local_frame;
        if (!are_points && m_setting_.scan.convert_to_points) {
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
                if (m_scan_frame_3d_->GetNumHitRays() == 0) {
                    ROS_WARN("No valid points. Skipping update.");
                    return;
                }
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
        if (success && m_received_colors_ && m_bhm_ && are_points) {
            // Fold per-point colors into surface voxels after the update.
            MatrixDX points_world;
            if (in_local) {
                points_world.noalias() = (rotation * scan).colwise() + translation;
            } else {
                points_world = scan;
            }
            const long n = points_world.cols();
            (void) m_bhm_->PaintVoxels(
                points_world,
                m_point_colors_.leftCols(n),
                m_setting_.paint_surface.overwrite,
                /*parallel=*/true);
        }
        {
            double time_budget_us = 1e6 / m_sdf_mapping_cfg_->update_hz;  // us
            ERL_BLOCK_TIMER_MSG("Update SDF GPs");
            m_sdf_mapping_->UpdateGpSdf(time_budget_us - surf_mapping_time * 1000);
        }
        auto t2 = ros::WallTime::now();
        m_msg_update_time_.data = (t2 - t1).toSec();
        m_pub_update_time_.publish(m_msg_update_time_);
        ROS_INFO("Update fps: %f", 1.0 / m_msg_update_time_.data);
        if (!success) { ROS_WARN("Failed to update SDF mapping"); }

        erl::common::BlockTimerRecords::PrintRecords();
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

        erl::common::BlockTimerRecords::PrintRecords();

        return true;
    }

    template<typename BhmType>
    void
    FillBhmOccQueryResponse(
        const std::shared_ptr<BhmType> &bhm,
        const MatrixDX &positions,
        int n,
        const std::string &mode,
        bool compute_gradient,
        erl_gp_sdf_msgs::OccQuery::Response &res) {

        VectorX prob_occupied(n);
        MatrixDX gradients(Dim, n);

        bhm->Predict(
            positions,
            /*logodd=*/(mode == "logodd"),
            /*compute_gradient=*/compute_gradient,
            /*gradient_with_sigmoid=*/false,
            /*parallel=*/true,
            prob_occupied,
            gradients);

        // Fill results
        res.results.resize(n);
        for (int i = 0; i < n; ++i) { res.results[i] = static_cast<double>(prob_occupied[i]); }

        // Fill gradients if requested
        res.gradients.clear();
        if (compute_gradient) {
            res.gradients.resize(n);
            if constexpr (Dim == 2) {
                for (int i = 0; i < n; ++i) {
                    res.gradients[i].x = static_cast<double>(gradients(0, i));
                    res.gradients[i].y = static_cast<double>(gradients(1, i));
                    res.gradients[i].z = 0.0;
                }
            } else {
                for (int i = 0; i < n; ++i) {
                    res.gradients[i].x = static_cast<double>(gradients(0, i));
                    res.gradients[i].y = static_cast<double>(gradients(1, i));
                    res.gradients[i].z = static_cast<double>(gradients(2, i));
                }
            }
        }

        res.success = true;
    }

    bool
    CallbackOccQuery(
        erl_gp_sdf_msgs::OccQuery::Request &req,
        erl_gp_sdf_msgs::OccQuery::Response &res) {

        const auto n = static_cast<int>(req.query_points.size());
        const std::string &mode = req.mode;
        const bool compute_gradient = req.compute_gradient;

        // Validate mode
        if (mode != "logodd" && mode != "prob") {
            res.success = false;
            res.reason = "invalid mode '" + mode + "', expected 'logodd' or 'prob'";
            return true;
        }

        // Parse query points (same pattern as CallbackSdfQuery)
        Eigen::Map<const Eigen::Matrix3Xd> positions_org(
            reinterpret_cast<const double *>(req.query_points.data()),
            3,
            n);
        MatrixDX positions = positions_org.topRows<Dim>().template cast<Dtype>();

        res.dim = Dim;

        // Try BayesianHilbertSurfaceMapping
        if (m_bhm_) {
            FillBhmOccQueryResponse(m_bhm_, positions, n, mode, compute_gradient, res);
            return true;
        }

        // Fallback: GpOccSurfaceMapping (tree-based query)
        auto gp_occ = std::dynamic_pointer_cast<GpOccSurfaceMapping>(m_surface_mapping_);
        if (gp_occ) {
            if (compute_gradient) {
                res.success = false;
                res.reason = "compute_gradient is not supported for GpOccSurfaceMapping";
                return true;
            }

            auto tree = gp_occ->GetTree();
            if (!tree) {
                res.success = false;
                res.reason = "GpOccSurfaceMapping has no occupancy tree";
                return true;
            }

            res.results.resize(n);
            const bool is_logodd = (mode == "logodd");
#pragma omp parallel for default(none) shared(res, tree, positions, n, is_logodd)
            for (int i = 0; i < n; ++i) {
                const auto *node = [&]() {
                    if constexpr (Dim == 2) {
                        return tree->Search(positions(0, i), positions(1, i));
                    } else {
                        return tree->Search(positions(0, i), positions(1, i), positions(2, i));
                    }
                }();
                if (node == nullptr) {
                    // Unknown space
                    if (is_logodd) {
                        res.results[i] = 0.0;
                    } else {
                        res.results[i] = 0.5;
                    }
                } else {
                    if (is_logodd) {
                        res.results[i] = static_cast<double>(node->GetLogOdds());
                    } else {
                        res.results[i] = static_cast<double>(node->GetOccupancy());
                    }
                }
            }

            res.success = true;
            return true;
        }

        res.success = false;
        res.reason = "no occupancy mapping frontend available";
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
        using Serializer = serialization::Serialization<GpSdfMapping>;
        res.success = Serializer::Read(map_file, m_sdf_mapping_.get());
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
        using Serializer = serialization::Serialization<GpSdfMapping>;
        res.success = Serializer::Write(map_file, m_sdf_mapping_.get());
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
        std::vector<Color> face_colors;
        const bool has_color = m_setting_.paint_surface.color_from_scan || m_paint_surface_enabled_;
        try {
            if (has_color) {
                res.success = m_surface_mapping_->GetMesh(false, vertices, faces, face_colors);
            } else {
                res.success = m_surface_mapping_->GetMesh(false, vertices, faces);
            }
        } catch (const std::exception &e) {
            ROS_WARN("Failed to get mesh: %s", e.what());
            res.success = false;
            return false;
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
            if (has_color && face_colors.size() == faces.size() && !faces.empty()) {
                // Open3D TriangleMesh carries per-vertex colors, not per-face. Average
                // incident face colors at each vertex.
                mesh.vertex_colors_.assign(vertices.size(), Eigen::Vector3d::Zero());
                std::vector<uint32_t> counts(vertices.size(), 0);
                for (size_t i = 0; i < faces.size(); ++i) {
                    const Eigen::Vector3d c(
                        face_colors[i][0] / 255.0,
                        face_colors[i][1] / 255.0,
                        face_colors[i][2] / 255.0);
                    for (int k = 0; k < 3; ++k) {
                        const auto v = faces[i](k);
                        mesh.vertex_colors_[v] += c;
                        ++counts[v];
                    }
                }
                for (size_t v = 0; v < vertices.size(); ++v) {
                    if (counts[v] > 0) {
                        mesh.vertex_colors_[v] /= static_cast<double>(counts[v]);
                    } else {
                        mesh.vertex_colors_[v] = Eigen::Vector3d(1.0, 1.0, 1.0);
                    }
                }
            }
            constexpr bool write_ascii = false;
            constexpr bool compressed = false;
            constexpr bool write_vertex_normals = true;
            const bool write_vertex_colors = !mesh.vertex_colors_.empty();
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
        erl::geometry::SaveToOccupancyTreeMsg<Dtype>(
            m_tree_,
            m_surface_mapping_->GetScaling(),
            m_setting_.publish_tree.binary,
            m_msg_tree_);
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

    void
    CallbackPublishMesh(const ros::TimerEvent & /* event */) {
        if (m_pub_mesh_.getNumSubscribers() == 0) { return; }  // no subscribers
        if (!m_surface_mapping_) { return; }

        std::vector<VectorD> vertices;
        std::vector<Eigen::Vector<int, Dim>> faces;
        std::vector<Color> face_colors;
        try {
            if (m_setting_.paint_surface.color_from_scan || m_paint_surface_enabled_) {
                if (!m_surface_mapping_->GetMesh(false, vertices, faces, face_colors)) { return; }
            } else {
                if (!m_surface_mapping_->GetMesh(false, vertices, faces)) { return; }
            }
        } catch (const std::exception &e) {
            ROS_WARN("Failed to get mesh: %s", e.what());
            return;
        }

        auto &msg = m_msg_mesh_;
        msg.header.stamp = ros::Time::now();
        msg.dim = Dim;

        // populate vertices
        msg.mesh.vertices.resize(vertices.size());
        for (size_t i = 0; i < vertices.size(); ++i) {
            msg.mesh.vertices[i].x = static_cast<double>(vertices[i][0]);
            msg.mesh.vertices[i].y = static_cast<double>(vertices[i][1]);
            msg.mesh.vertices[i].z = (Dim == 3) ? static_cast<double>(vertices[i][2]) : 0.0;
        }

        // populate faces (triangles for 3D, line segments for 2D)
        msg.mesh.triangles.resize(faces.size());
        for (size_t i = 0; i < faces.size(); ++i) {
            msg.mesh.triangles[i].vertex_indices[0] = static_cast<uint32_t>(faces[i][0]);
            msg.mesh.triangles[i].vertex_indices[1] = static_cast<uint32_t>(faces[i][1]);
            msg.mesh.triangles[i].vertex_indices[2] =
                (Dim == 3) ? static_cast<uint32_t>(faces[i][2]) : 0;
        }

        // populate face colors if available
        msg.vertex_colors.clear();
        if (!face_colors.empty()) {
            msg.face_colors.resize(face_colors.size());
            for (size_t i = 0; i < face_colors.size(); ++i) {
                msg.face_colors[i].r = static_cast<float>(face_colors[i][0]) / 255.0f;
                msg.face_colors[i].g = static_cast<float>(face_colors[i][1]) / 255.0f;
                msg.face_colors[i].b = static_cast<float>(face_colors[i][2]) / 255.0f;
                msg.face_colors[i].a = static_cast<float>(face_colors[i][3]) / 255.0f;
            }
        } else {
            msg.face_colors.clear();
        }

        m_pub_mesh_.publish(msg);
    }

    void
    CallbackPublishOccupancyGrid(const ros::TimerEvent & /* event */) {
        if constexpr (Dim == 3) {

            if (m_pub_occupancy_grid_.getNumSubscribers() == 0) { return; }
            if (!m_surface_mapping_) { return; }

            using GridMapInfo = erl::common::GridMapInfo2D<Dtype>;

            std::vector<int8_t> occupancy_data;
            GridMapInfo grid_info(
                Eigen::Vector2<Dtype>(0, 0),
                Eigen::Vector2<Dtype>(1, 1),
                Eigen::Vector2i(1, 1));

            if (!m_height_map_projector_) { return; }
            auto bhm =
                m_bhm_
                    ? m_bhm_
                    : std::dynamic_pointer_cast<BayesianHilbertSurfaceMapping>(m_surface_mapping_);
            if (!bhm) { return; }
            m_height_map_projector_->Update(*bhm);
            m_height_map_projector_->GetOccupancyGrid(occupancy_data, grid_info);

            if (occupancy_data.empty()) { return; }

            // Internal layout: rows index X, cols index Y, data[x * Ny + y] (X-major).
            // Nav layout:      width = X cells, height = Y cells, data[y * width + x] (Y-major).
            const uint32_t nx = static_cast<uint32_t>(grid_info.Shape()[0]);
            const uint32_t ny = static_cast<uint32_t>(grid_info.Shape()[1]);

            auto &msg = m_msg_occupancy_grid_;
            msg.header.stamp = ros::Time::now();
            msg.info.resolution = static_cast<float>(grid_info.Resolution()[0]);
            msg.info.width = nx;
            msg.info.height = ny;
            msg.info.origin.position.x = static_cast<double>(grid_info.Min()[0]);
            msg.info.origin.position.y = static_cast<double>(grid_info.Min()[1]);
            msg.info.origin.position.z = 0.0;
            msg.info.origin.orientation.w = 1.0;

            // Transpose: our[x * ny + y] -> nav[y * nx + x].
            msg.data.resize(static_cast<std::size_t>(nx) * ny);
            for (uint32_t x = 0; x < nx; ++x) {
                for (uint32_t y = 0; y < ny; ++y) {
                    msg.data[y * nx + x] = occupancy_data[x * ny + y];
                }
            }

            m_pub_occupancy_grid_.publish(msg);
        } else {
            return;
        }
    }
};

template<typename Dtype, int Dim>
int
Spin(ros::NodeHandle &nh) {
    SdfMappingNode<Dtype, Dim> node(nh);
    bool async_spin = false;
    nh.param<bool>("async_spin", async_spin, false);
    if (async_spin) {
        ros::AsyncSpinner spinner(0);  // use all available threads
        spinner.start();
        ros::waitForShutdown();
    } else {
        ros::spin();
    }
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
