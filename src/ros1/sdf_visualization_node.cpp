#include "erl_common/eigen.hpp"
#include "erl_common/yaml.hpp"
#include "erl_gp_sdf_msgs/SaveMap.h"
#include "erl_gp_sdf_msgs/SdfQuery.h"

#include <geometry_msgs/Vector3.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <filesystem>

using namespace erl::common;

struct SdfVisNodeConfig : public Yamlable<SdfVisNodeConfig> {
    // resolution of the grid map.
    double resolution = 0.1;
    // number of cells in the grid map along the x-axis.
    int x_cells = 101;
    // number of cells in the grid map along the y-axis.
    int y_cells = 101;
    // the x coordinate of the grid map origin.
    double x = 0.0;
    // the y coordinate of the grid map origin.
    double y = 0.0;
    // the z coordinate of the grid map origin.
    double z = 0.0;
    // if true, the gradient of the SDF is published.
    bool publish_gradient = false;
    // if true, the variance of the SDF is published.
    bool publish_sdf_variance = false;
    // if true, the variance of the SDF gradient is published.
    bool publish_gradient_variance = false;
    // if true, the covariance between the SDF and the gradient is published.
    bool publish_covariance = false;
    // if true, publish a grid map of the result.
    bool publish_grid_map = true;
    // if true, publish a point cloud of the result.
    bool publish_point_cloud = false;
    // the frequency of publishing the grid map.
    double publish_rate = 2;
    // if true, the grid map is moved with the frame.
    bool attached_to_frame = false;
    // the name of the frame to attach the grid map to.
    std::string attached_frame = "sdf_query_frame";
    // the name of the world frame, used when attached_to_frame is true.
    std::string world_frame = "map";
    // the name of the service to query the SDF.
    std::string sdf_query_service = "/sdf_mapping_node/sdf_query";
    // the name of the topic to publish the grid map.
    std::string map_topic = "sdf_grid_map";
    // the name of the topic to publish the point cloud.
    std::string point_cloud_topic = "sdf_point_cloud";
    // save query service
    std::string save_query_service = "sdf_save_query";

    ERL_REFLECT_SCHEMA(
        SdfVisNodeConfig,
        ERL_REFLECT_MEMBER(SdfVisNodeConfig, resolution),
        ERL_REFLECT_MEMBER(SdfVisNodeConfig, x_cells),
        ERL_REFLECT_MEMBER(SdfVisNodeConfig, y_cells),
        ERL_REFLECT_MEMBER(SdfVisNodeConfig, x),
        ERL_REFLECT_MEMBER(SdfVisNodeConfig, y),
        ERL_REFLECT_MEMBER(SdfVisNodeConfig, z),
        ERL_REFLECT_MEMBER(SdfVisNodeConfig, publish_gradient),
        ERL_REFLECT_MEMBER(SdfVisNodeConfig, publish_sdf_variance),
        ERL_REFLECT_MEMBER(SdfVisNodeConfig, publish_gradient_variance),
        ERL_REFLECT_MEMBER(SdfVisNodeConfig, publish_covariance),
        ERL_REFLECT_MEMBER(SdfVisNodeConfig, publish_grid_map),
        ERL_REFLECT_MEMBER(SdfVisNodeConfig, publish_point_cloud),
        ERL_REFLECT_MEMBER(SdfVisNodeConfig, publish_rate),
        ERL_REFLECT_MEMBER(SdfVisNodeConfig, attached_to_frame),
        ERL_REFLECT_MEMBER(SdfVisNodeConfig, attached_frame),
        ERL_REFLECT_MEMBER(SdfVisNodeConfig, world_frame),
        ERL_REFLECT_MEMBER(SdfVisNodeConfig, sdf_query_service),
        ERL_REFLECT_MEMBER(SdfVisNodeConfig, map_topic),
        ERL_REFLECT_MEMBER(SdfVisNodeConfig, point_cloud_topic),
        ERL_REFLECT_MEMBER(SdfVisNodeConfig, save_query_service));

    bool
    PostDeserialization() override {
        // check the parameters
        if (resolution <= 0) {
            ROS_WARN("Resolution must be positive");
            return false;
        }
        if (x_cells <= 0) {
            ROS_WARN("X cells must be positive");
            return false;
        }
        if (x_cells % 2 == 0) {
            x_cells += 1;
            ROS_WARN("X cells must be odd, set to %d", x_cells);
        }
        if (y_cells <= 0) {
            ROS_WARN("Y cells must be positive");
            return false;
        }
        if (y_cells % 2 == 0) {
            y_cells += 1;
            ROS_WARN("Y cells must be odd, set to %d", y_cells);
        }
        if (!publish_grid_map && !publish_point_cloud) {
            ROS_WARN("At least one of publish_grid_map or publish_point_cloud must be true");
            return false;
        }
        if (publish_rate <= 0) {
            ROS_WARN("Publish frequency must be positive");
            return false;
        }
        if (attached_to_frame) {
            if (attached_frame.empty()) {
                ROS_WARN("Attached frame is empty but attached_to_frame is true");
                return false;
            }
            if (world_frame.empty()) {
                ROS_WARN("World frame is empty but attached_to_frame is true");
                return false;
            }
        }
        if (sdf_query_service.empty()) {
            ROS_WARN("sdf_query_service is empty");
            return false;
        }
        if (map_topic.empty()) {
            ROS_WARN("map_topic is empty");
            return false;
        }
        if (point_cloud_topic.empty()) {
            ROS_WARN("point_cloud_topic is empty");
            return false;
        }
        if (save_query_service.empty()) {
            ROS_WARN("save_query_service is empty");
            return false;
        }
        return true;
    }
};

class SdfVisualizationNode {
    SdfVisNodeConfig m_setting_;
    ros::NodeHandle m_nh_;
    ros::ServiceClient m_sdf_client_;
    ros::Publisher m_pub_map_;
    ros::Publisher m_pub_pcd_;
    ros::ServiceServer m_srv_save_query_;
    ros::Timer m_timer_;
    tf2_ros::Buffer m_tf_buffer_;
    std::vector<geometry_msgs::Vector3> m_query_points_;
    ros::Time m_stamp_;
    grid_map::GridMap m_grid_map_;
    sensor_msgs::PointCloud2 m_cloud_;

public:
    SdfVisualizationNode(ros::NodeHandle &nh)
        : m_nh_(nh) {
        if (!LoadParameters()) {
            ROS_FATAL("Failed to load parameters");
            ros::shutdown();
            return;
        }

        InitQueryPoints();

        m_sdf_client_ =
            m_nh_.serviceClient<erl_gp_sdf_msgs::SdfQuery>(m_setting_.sdf_query_service);
        if (m_setting_.publish_grid_map) {
            m_pub_map_ = m_nh_.advertise<grid_map_msgs::GridMap>(m_setting_.map_topic, 1, true);
        }
        if (m_setting_.publish_point_cloud) {
            m_pub_pcd_ =
                m_nh_.advertise<sensor_msgs::PointCloud2>(m_setting_.point_cloud_topic, 1, true);
        }
        m_srv_save_query_ = m_nh_.advertiseService(
            m_setting_.save_query_service,
            &SdfVisualizationNode::CallbackSaveQuery,
            this);
        m_grid_map_.setGeometry(
            grid_map::Length(
                static_cast<double>(m_setting_.x_cells) * m_setting_.resolution,
                static_cast<double>(m_setting_.y_cells) * m_setting_.resolution),
            m_setting_.resolution,
            grid_map::Position(m_setting_.x, m_setting_.y));
        if (m_setting_.attached_to_frame) {
            m_cloud_.header.frame_id = m_setting_.attached_frame;
            m_grid_map_.setFrameId(m_setting_.attached_frame);
        } else {
            m_cloud_.header.frame_id = m_setting_.world_frame;
            m_grid_map_.setFrameId(m_setting_.world_frame);
        }
        m_cloud_.header.seq = -1;
        m_cloud_.height = 1;
        m_cloud_.is_bigendian = false;
        m_cloud_.is_dense = false;
        m_timer_ =
            m_nh_.createTimer(ros::Duration(0.5), &SdfVisualizationNode::CallbackTimer, this);
        ROS_INFO("SdfVisualizationNode initialized");
    }

private:
    bool
    LoadParameters() {
        return m_setting_.LoadFromRos1(m_nh_, "");
    }

    void
    InitQueryPoints() {
        m_query_points_.clear();
        m_query_points_.reserve(m_setting_.x_cells * m_setting_.y_cells);
        int half_x = (m_setting_.x_cells - 1) / 2;
        int half_y = (m_setting_.y_cells - 1) / 2;
        for (int j = half_y; j >= -half_y; --j) {      // y-axis
            for (int i = half_x; i >= -half_x; --i) {  // x-axis (column major)
                geometry_msgs::Vector3 p;
                p.x = static_cast<double>(i) * m_setting_.resolution + m_setting_.x;
                p.y = static_cast<double>(j) * m_setting_.resolution + m_setting_.y;
                p.z = m_setting_.z;
                m_query_points_.push_back(p);
            }
        }
        ROS_INFO(
            "Query points initialized, %d x %d points",
            m_setting_.x_cells,
            m_setting_.y_cells);
    }

    bool
    MakeSdfQuery(erl_gp_sdf_msgs::SdfQuery &srv) {
        if (!m_sdf_client_.exists()) {
            ROS_WARN("SDF query service %s not available", m_setting_.sdf_query_service.c_str());
            return false;
        }

        geometry_msgs::TransformStamped transform_stamped;

        if (m_setting_.attached_to_frame) {
            try {
                transform_stamped = m_tf_buffer_.lookupTransform(
                    m_setting_.world_frame,
                    m_setting_.attached_frame,
                    ros::Time(0),
                    ros::Duration(1.0));
            } catch (tf2::TransformException &ex) {
                ROS_WARN(ex.what());
                return false;
            }
            srv.request.query_points.clear();
            srv.request.query_points.reserve(m_query_points_.size());
            Eigen::Isometry3d transform = tf2::transformToEigen(transform_stamped.transform);
            for (auto &ps: m_query_points_) {
                // transform ps from the attached frame to the world frame
                Eigen::Vector4d p_world = transform * Eigen::Vector4d(ps.x, ps.y, ps.z, 1.0);
                geometry_msgs::Vector3 p;
                p.x = p_world.x();
                p.y = p_world.y();
                p.z = p_world.z();
                srv.request.query_points.emplace_back(std::move(p));
            }
            m_stamp_ = transform_stamped.header.stamp;
        } else {
            srv.request.query_points = m_query_points_;
            m_stamp_ = ros::Time::now();
        }

        if (!m_sdf_client_.call(srv)) {
            ROS_WARN("Failed to call %s", m_sdf_client_.getService().c_str());
            return false;
        }

        auto &ans = srv.response;
        if (!ans.success) {
            ROS_WARN("SDF query failed");
            return false;
        }

        const auto n = static_cast<int>(ans.signed_distances.size());
        const auto map_size = m_grid_map_.getSize();
        if (const auto m = map_size[0] * map_size[1]; n != m) {
            ROS_WARN("Query points size %d does not match map size %d", n, m);
            return false;
        }

        return true;
    }

    void
    CallbackTimer(const ros::TimerEvent &) {
        erl_gp_sdf_msgs::SdfQuery srv;
        if (!MakeSdfQuery(srv)) { return; }

        auto &ans = srv.response;

        // 1. copy the result to the grid map / the point cloud
        // 2. publish the grid map / point cloud

        if (m_setting_.publish_grid_map) {
            m_grid_map_.setTimestamp(m_stamp_.toNSec());
            LoadResponseToGridMap(ans);
            grid_map_msgs::GridMap msg;
            grid_map::GridMapRosConverter::toMessage(m_grid_map_, msg);
            m_pub_map_.publish(msg);
        }

        if (m_setting_.publish_point_cloud) {
            m_cloud_.header.stamp = m_stamp_;
            m_cloud_.header.seq += 1;
            m_cloud_.data.clear();
            InitializePointCloudFields(ans);
            LoadResponseToPointCloud(ans);
            m_pub_pcd_.publish(m_cloud_);
        }
    }

    bool
    CallbackSaveQuery(
        erl_gp_sdf_msgs::SaveMap::Request &req,
        erl_gp_sdf_msgs::SaveMap::Response &res) {
        erl_gp_sdf_msgs::SdfQuery srv;
        if (req.name.empty()) {
            ROS_WARN("Save query name is empty");
            res.success = false;
            return false;
        }
        if (!MakeSdfQuery(srv)) {
            res.success = false;
            return false;
        }

        if (!std::filesystem::exists(req.name)) { std::filesystem::create_directories(req.name); }
        if (!std::filesystem::is_directory(req.name)) {
            ROS_WARN("Save query path %s is not a directory", req.name.c_str());
            res.success = false;
            return false;
        }

        auto &ans = srv.response;
        std::filesystem::path dir_path(req.name);

        Eigen::Map<const Eigen::VectorXd> sdf(
            ans.signed_distances.data(),
            ans.signed_distances.size());
        const auto sdf_filepath = dir_path / "sdf.bin";
        std::ofstream ofs_sdf(sdf_filepath, std::ios::binary);
        if (!erl::common::SaveEigenMapToBinaryStream(ofs_sdf, sdf)) {
            ROS_WARN("Failed to save sdf to %s", sdf_filepath.c_str());
            res.success = false;
            return false;
        }

        if (ans.compute_gradient) {
            Eigen::Map<const Eigen::MatrixXd> gradients(
                reinterpret_cast<const double *>(ans.gradients.data()),
                ans.dim,
                ans.signed_distances.size());
            const auto gradients_filepath = dir_path / "gradients.bin";
            std::ofstream ofs_grad(gradients_filepath, std::ios::binary);
            if (!erl::common::SaveEigenMapToBinaryStream(ofs_grad, gradients)) {
                ROS_WARN("Failed to save gradients to %s", gradients_filepath.c_str());
                res.success = false;
                return false;
            }
        }

        if (!ans.variances.empty()) {
            Eigen::Map<const Eigen::VectorXd> variances(
                reinterpret_cast<const double *>(ans.variances.data()),
                ans.variances.size());
            const auto variances_filepath = dir_path / "variances.bin";
            std::ofstream ofs_var(variances_filepath, std::ios::binary);
            if (!erl::common::SaveEigenMapToBinaryStream(ofs_var, variances)) {
                ROS_WARN("Failed to save variances to %s", variances_filepath.c_str());
                res.success = false;
                return false;
            }
        }

        if (!ans.covariances.empty()) {
            Eigen::Map<const Eigen::MatrixXd> covariances(
                reinterpret_cast<const double *>(ans.covariances.data()),
                ans.dim * (ans.dim + 1) / 2,
                ans.signed_distances.size());
            const auto covariances_filepath = dir_path / "covariances.bin";
            std::ofstream ofs_cov(covariances_filepath, std::ios::binary);
            if (!erl::common::SaveEigenMapToBinaryStream(ofs_cov, covariances)) {
                ROS_WARN("Failed to save covariances to %s", covariances_filepath.c_str());
                res.success = false;
                return false;
            }
        }

        res.success = true;
        return true;
    }

    void
    InitializePointCloudFields(const erl_gp_sdf_msgs::SdfQuery::Response &ans) {
        if (!m_cloud_.fields.empty()) { return; }
        // initialize point cloud fields
        m_cloud_.fields.reserve(14);
        sensor_msgs::PointField field;
        field.datatype = sensor_msgs::PointField::FLOAT32;

        field.name = "x";
        field.offset = 0;
        m_cloud_.fields.push_back(field);  // x
        field.name = "y";
        field.offset += 4;
        m_cloud_.fields.push_back(field);  // y
        field.name = "z";
        field.offset += 4;
        m_cloud_.fields.push_back(field);  // z
        field.name = "sdf";
        field.offset += 4;
        m_cloud_.fields.push_back(field);  // sdf
        field.name = "var_sdf";
        field.offset += 4;
        m_cloud_.fields.push_back(field);  // sdf variance
        if (m_setting_.publish_gradient) {
            field.name = "gradient_x";
            field.offset += 4;
            m_cloud_.fields.push_back(field);  // gradient_x
            field.name = "gradient_y";
            field.offset += 4;
            m_cloud_.fields.push_back(field);  // gradient_y
            field.name = "gradient_z";
            field.offset += 4;
            m_cloud_.fields.push_back(field);  // gradient_z
        }
        if (m_setting_.publish_gradient_variance && ans.compute_gradient_variance) {
            field.name = "var_gradient_x";
            field.offset += 4;
            m_cloud_.fields.push_back(field);  // var_gradient_x
            field.name = "var_gradient_y";
            field.offset += 4;
            m_cloud_.fields.push_back(field);  // var_gradient_y
            if (ans.dim == 3) {
                field.name = "var_gradient_z";
                field.offset += 4;
                m_cloud_.fields.push_back(field);  // var_gradient_z
            }
        }
        if (m_setting_.publish_covariance && ans.compute_covariance) {
            if (ans.dim == 2) {
                field.name = "cov_gx_sdf";
                field.offset += 4;
                m_cloud_.fields.push_back(field);
                field.name = "cov_gy_sdf";
                field.offset += 4;
                m_cloud_.fields.push_back(field);
                field.name = "cov_gy_gx";
                field.offset += 4;
                m_cloud_.fields.push_back(field);
            } else if (ans.dim == 3) {
                field.name = "cov_gx_sdf";
                field.offset += 4;
                m_cloud_.fields.push_back(field);
                field.name = "cov_gy_sdf";
                field.offset += 4;
                m_cloud_.fields.push_back(field);
                field.name = "cov_gz_sdf";
                field.offset += 4;
                m_cloud_.fields.push_back(field);
                field.name = "cov_gy_gx";
                field.offset += 4;
                m_cloud_.fields.push_back(field);
                field.name = "cov_gz_gx";
                field.offset += 4;
                m_cloud_.fields.push_back(field);
                field.name = "cov_gz_gy";
                field.offset += 4;
                m_cloud_.fields.push_back(field);
            } else {
                ROS_WARN("Unknown dimension %d", ans.dim);
                return;
            }
        }
        m_cloud_.point_step = field.offset + 4;
    }

    void
    LoadResponseToGridMap(const erl_gp_sdf_msgs::SdfQuery::Response &ans) {
        if (!m_setting_.publish_grid_map) { return; }
        const auto map_size = m_grid_map_.getSize();
        const auto n = static_cast<int>(ans.signed_distances.size());
        if (const auto m = map_size[0] * map_size[1]; n != m) {
            ROS_WARN("Query points size %d does not match map size %d", n, m);
            return;
        }

        // SDF
        Eigen::MatrixXf sdf_map =
            Eigen::Map<const Eigen::MatrixXd>(ans.signed_distances.data(), map_size[0], map_size[1])
                .cast<float>();
        m_grid_map_.add("elevation", sdf_map);  // add to avoid rviz crash without elevation layer
        m_grid_map_.add("sdf", sdf_map);
        m_grid_map_.setBasicLayers({"sdf"});
        // gradient
        if (m_setting_.publish_gradient) {  // dim layers
            if (ans.compute_gradient) {
                Eigen::Map<const Eigen::MatrixXd> gradients(
                    reinterpret_cast<const double *>(ans.gradients.data()),
                    3,
                    n);
                static const char *gradient_names[3] = {"gradient_x", "gradient_y", "gradient_z"};
                for (int i = 0; i < ans.dim; ++i) {
                    Eigen::VectorXf grad_i = gradients.row(i).transpose().cast<float>();
                    Eigen::Map<Eigen::MatrixXf> grad_map(grad_i.data(), map_size[0], map_size[1]);
                    m_grid_map_.add(gradient_names[i], grad_map);
                }
            } else {
                ROS_WARN("Gradient is not computed");
            }
        }
        Eigen::Map<const Eigen::MatrixXd> variances(
            reinterpret_cast<const double *>(ans.variances.data()),
            ans.compute_gradient_variance ? ans.dim + 1 : 1,
            n);
        // SDF variance
        if (m_setting_.publish_sdf_variance) {  // 1 layer
            if (ans.compute_gradient_variance) {
                Eigen::VectorXf var_sdf = variances.row(0).transpose().cast<float>().unaryExpr(
                    [](float v) -> float { return v < 1.0e5 ? v : NAN; });
                Eigen::Map<Eigen::MatrixXf> sdf_variance_map(
                    var_sdf.data(),
                    map_size[0],
                    map_size[1]);
                m_grid_map_.add("var_sdf", sdf_variance_map);
            } else {
                m_grid_map_.add(
                    "var_sdf",
                    Eigen::Map<const Eigen::MatrixXd>(
                        ans.variances.data(),
                        map_size[0],
                        map_size[1])
                        .cast<float>()
                        .unaryExpr([](float v) -> float { return v < 1.0e5 ? v : NAN; }));
            }
            m_grid_map_.setBasicLayers({"var_sdf"});
        }
        // gradient variance
        if (m_setting_.publish_gradient_variance) {  // dim layers
            if (ans.compute_gradient_variance) {
                static const char *gradient_variance_names[3] = {
                    "var_gradient_x",
                    "var_gradient_y",
                    "var_gradient_z"};
                for (int i = 0; i < ans.dim; ++i) {
                    Eigen::VectorXf var_grad_i = variances.row(i + 1).transpose().cast<float>();
                    Eigen::Map<Eigen::MatrixXf> grad_variance_map(
                        var_grad_i.data(),
                        map_size[0],
                        map_size[1]);
                    m_grid_map_.add(gradient_variance_names[i], grad_variance_map);
                }
            } else {
                ROS_WARN("Gradient variance is not computed");
            }
        }
        // covariance
        if (m_setting_.publish_covariance) {  // dim * (dim + 1) / 2 layers
            if (ans.compute_covariance) {
                Eigen::Map<const Eigen::MatrixXd> covariances(
                    ans.covariances.data(),
                    ans.dim * (ans.dim + 1) / 2,
                    n);
                if (ans.dim == 2) {
                    static const char *covariance_names[3] = {
                        "cov_gx_sdf",
                        "cov_gy_sdf",
                        "cov_gy_gx"};
                    for (long i = 0; i < covariances.rows(); ++i) {
                        Eigen::VectorXf covariance_i = covariances.row(i).transpose().cast<float>();
                        Eigen::Map<Eigen::MatrixXf> covariance_map(
                            covariance_i.data(),
                            map_size[0],
                            map_size[1]);
                        m_grid_map_.add(covariance_names[i], covariance_map);
                    }
                } else if (ans.dim == 3) {
                    static const char *covariance_names[6] = {
                        "cov_gx_sdf",
                        "cov_gy_sdf",
                        "cov_gz_sdf",
                        "cov_gy_gx",
                        "cov_gz_gx",
                        "cov_gz_gy"};
                    for (long i = 0; i < covariances.rows(); ++i) {
                        Eigen::VectorXf covariance_i = covariances.row(i).transpose().cast<float>();
                        Eigen::Map<Eigen::MatrixXf> covariance_map(
                            covariance_i.data(),
                            map_size[0],
                            map_size[1]);
                        m_grid_map_.add(covariance_names[i], covariance_map);
                    }
                } else {
                    ROS_WARN("Unknown dimension %d", ans.dim);
                }
            } else {
                ROS_WARN("Covariance is not computed");
            }
        }
    }

    void
    LoadResponseToPointCloud(const erl_gp_sdf_msgs::SdfQuery::Response &ans) {
        if (!m_setting_.publish_point_cloud) { return; }
        const auto map_size = m_grid_map_.getSize();
        const auto n = static_cast<int>(ans.signed_distances.size());
        if (const auto m = map_size[0] * map_size[1]; n != m) {
            ROS_WARN("Query points size %d does not match map size %d", n, m);
            return;
        }

        m_cloud_.data.resize(m_cloud_.point_step * n);
        const double *dummy_ptr = ans.signed_distances.data();
        Eigen::Map<Eigen::MatrixXf> data_out(
            reinterpret_cast<float *>(m_cloud_.data.data()),
            m_cloud_.point_step / 4,
            n);
        Eigen::Map<const Eigen::MatrixXd> gradients(
            m_setting_.publish_gradient && ans.compute_gradient
                ? reinterpret_cast<const double *>(ans.gradients.data())
                : dummy_ptr,
            3,
            n);
        Eigen::Map<const Eigen::MatrixXd> variances(
            reinterpret_cast<const double *>(ans.variances.data()),
            ans.compute_gradient_variance ? ans.dim + 1 : 1,
            n);
        Eigen::Map<const Eigen::MatrixXd> covariances(
            ans.compute_covariance ? ans.covariances.data() : dummy_ptr,
            ans.dim * (ans.dim + 1) / 2,
            n);

        int j = 0;  // index for the point cloud data
        for (int i = 0; i < n; ++i) {
            const double *var_ptr = variances.col(i).data();
            if (var_ptr[0] >= 1.0e5) { continue; }  // invalid SDF value, skip this point
            float *out = data_out.col(j++).data();
            out[0] = static_cast<float>(m_query_points_[i].x);     // x
            out[1] = static_cast<float>(m_query_points_[i].y);     // y
            out[2] = static_cast<float>(m_query_points_[i].z);     // z
            out[3] = static_cast<float>(ans.signed_distances[i]);  // sdf
            out[4] = static_cast<float>(var_ptr[0]);               // var_sdf
            int fid = 5;
            if (m_setting_.publish_gradient && ans.compute_gradient) {
                out[fid++] = static_cast<float>(gradients(0, i));  // gradient_x
                out[fid++] = static_cast<float>(gradients(1, i));  // gradient_y
                out[fid++] = static_cast<float>(gradients(2, i));  // gradient_z
            }
            if (m_setting_.publish_gradient_variance && ans.compute_gradient_variance) {
                out[fid++] = static_cast<float>(var_ptr[1]);  // var_gradient_x
                out[fid++] = static_cast<float>(var_ptr[2]);  // var_gradient_y
                if (ans.dim == 3) {                           // var_gradient_z
                    out[fid++] = static_cast<float>(var_ptr[3]);
                }
            }
            if (m_setting_.publish_covariance && ans.compute_covariance) {
                const double *cov_ptr = covariances.col(i).data();
                if (ans.dim == 2) {
                    out[fid++] = static_cast<float>(cov_ptr[0]);  // cov_gx_sdf
                    out[fid++] = static_cast<float>(cov_ptr[1]);  // cov_gy_sdf
                    out[fid++] = static_cast<float>(cov_ptr[2]);  // cov_gy_gx
                } else if (ans.dim == 3) {
                    out[fid++] = static_cast<float>(cov_ptr[0]);  // cov_gx_sdf
                    out[fid++] = static_cast<float>(cov_ptr[1]);  // cov_gy_sdf
                    out[fid++] = static_cast<float>(cov_ptr[2]);  // cov_gz_sdf
                    out[fid++] = static_cast<float>(cov_ptr[3]);  // cov_gy_gx
                    out[fid++] = static_cast<float>(cov_ptr[4]);  // cov_gz_gx
                    out[fid++] = static_cast<float>(cov_ptr[5]);  // cov_gz_gy
                } else {
                    ROS_WARN("Unknown dimension %d", ans.dim);
                }
            }
        }

        m_cloud_.width = j;
        m_cloud_.row_step = m_cloud_.point_step * j;
        m_cloud_.data.resize(m_cloud_.row_step);
        for (auto &field: m_cloud_.fields) { field.count = j; }
    }
};

int
main(int argc, char **argv) {
    ros::init(argc, argv, "sdf_visualization_node");
    ros::NodeHandle nh("~");  // ~: shorthand for the private namespace
    SdfVisualizationNode node(nh);
    ros::spin();
    return 0;
}
