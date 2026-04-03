#include "erl_common/eigen.hpp"
#include "erl_common/yaml.hpp"
#include "erl_gp_sdf_msgs/OccQuery.h"
#include "erl_gp_sdf_msgs/SaveMap.h"

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <filesystem>

using namespace erl::common;

struct OccVisNodeConfig : public Yamlable<OccVisNodeConfig> {
    double resolution = 0.1;
    int x_cells = 101;
    int y_cells = 101;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    // mode: "logodd" or "prob"
    std::string mode = "prob";
    bool compute_gradient = false;
    bool publish_grid_map = true;
    // publish nav_msgs::OccupancyGrid (only meaningful for "prob" mode)
    bool publish_occupancy_grid = true;
    double publish_rate = 2;
    bool attached_to_frame = false;
    std::string attached_frame = "occ_query_frame";
    std::string world_frame = "map";
    std::string occ_query_service = "/sdf_mapping_node/occ_query";
    std::string save_query_service = "occ_save_query";
    std::string trigger_query_service = "occ_trigger_query";
    std::string map_topic = "occ_grid_map";
    std::string occupancy_grid_topic = "occupancy_grid";

    ERL_REFLECT_SCHEMA(
        OccVisNodeConfig,
        ERL_REFLECT_MEMBER(OccVisNodeConfig, resolution),
        ERL_REFLECT_MEMBER(OccVisNodeConfig, x_cells),
        ERL_REFLECT_MEMBER(OccVisNodeConfig, y_cells),
        ERL_REFLECT_MEMBER(OccVisNodeConfig, x),
        ERL_REFLECT_MEMBER(OccVisNodeConfig, y),
        ERL_REFLECT_MEMBER(OccVisNodeConfig, z),
        ERL_REFLECT_MEMBER(OccVisNodeConfig, mode),
        ERL_REFLECT_MEMBER(OccVisNodeConfig, compute_gradient),
        ERL_REFLECT_MEMBER(OccVisNodeConfig, publish_grid_map),
        ERL_REFLECT_MEMBER(OccVisNodeConfig, publish_occupancy_grid),
        ERL_REFLECT_MEMBER(OccVisNodeConfig, publish_rate),
        ERL_REFLECT_MEMBER(OccVisNodeConfig, attached_to_frame),
        ERL_REFLECT_MEMBER(OccVisNodeConfig, attached_frame),
        ERL_REFLECT_MEMBER(OccVisNodeConfig, world_frame),
        ERL_REFLECT_MEMBER(OccVisNodeConfig, occ_query_service),
        ERL_REFLECT_MEMBER(OccVisNodeConfig, save_query_service),
        ERL_REFLECT_MEMBER(OccVisNodeConfig, trigger_query_service),
        ERL_REFLECT_MEMBER(OccVisNodeConfig, map_topic),
        ERL_REFLECT_MEMBER(OccVisNodeConfig, occupancy_grid_topic));

    bool
    PostDeserialization() override {
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
        if (mode != "logodd" && mode != "prob") {
            ROS_WARN("mode must be 'logodd' or 'prob', got '%s'", mode.c_str());
            return false;
        }
        if (!publish_grid_map && !publish_occupancy_grid) {
            ROS_WARN("At least one of publish_grid_map or publish_occupancy_grid must be true");
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
        if (occ_query_service.empty()) {
            ROS_WARN("occ_query_service is empty");
            return false;
        }
        if (save_query_service.empty()) {
            ROS_WARN("save_query_service is empty");
            return false;
        }
        if (trigger_query_service.empty()) {
            ROS_WARN("trigger_query_service is empty");
            return false;
        }
        if (map_topic.empty()) {
            ROS_WARN("Map topic is empty");
            return false;
        }
        if (occupancy_grid_topic.empty()) {
            ROS_WARN("Occupancy grid topic is empty");
            return false;
        }
        return true;
    }
};

class OccVisualizationNode {
    OccVisNodeConfig m_setting_;
    ros::NodeHandle m_nh_;
    ros::ServiceClient m_occ_client_;
    ros::ServiceServer m_srv_save_query_;
    ros::ServiceServer m_srv_trigger_query_;
    ros::Publisher m_pub_map_;
    ros::Publisher m_pub_occ_grid_;
    ros::Timer m_timer_;
    tf2_ros::Buffer m_tf_buffer_;
    tf2_ros::TransformListener m_tf_listener_{m_tf_buffer_};
    std::vector<geometry_msgs::Vector3> m_query_points_;
    ros::Time m_stamp_;
    grid_map::GridMap m_grid_map_;
    nav_msgs::OccupancyGrid m_occ_grid_;

public:
    explicit OccVisualizationNode(ros::NodeHandle &nh)
        : m_nh_(nh) {

        if (!m_setting_.LoadFromRos1(m_nh_, "")) {
            ROS_FATAL("Failed to load parameters");
            ros::shutdown();
            return;
        }

        ROS_INFO("Loaded node parameters:\n%s", m_setting_.AsYamlString().c_str());

        InitQueryPoints();

        m_occ_client_ =
            m_nh_.serviceClient<erl_gp_sdf_msgs::OccQuery>(m_setting_.occ_query_service);
        m_srv_save_query_ = m_nh_.advertiseService(
            m_setting_.save_query_service,
            &OccVisualizationNode::CallbackSaveQuery,
            this);
        m_srv_trigger_query_ = m_nh_.advertiseService(
            m_setting_.trigger_query_service,
            &OccVisualizationNode::CallbackTriggerQuery,
            this);
        if (m_setting_.publish_grid_map) {
            m_pub_map_ = m_nh_.advertise<grid_map_msgs::GridMap>(m_setting_.map_topic, 1, true);
        }
        if (m_setting_.publish_occupancy_grid) {
            m_pub_occ_grid_ =
                m_nh_.advertise<nav_msgs::OccupancyGrid>(m_setting_.occupancy_grid_topic, 1, true);
        }
        m_grid_map_.setGeometry(
            grid_map::Length(
                static_cast<double>(m_setting_.x_cells) * m_setting_.resolution,
                static_cast<double>(m_setting_.y_cells) * m_setting_.resolution),
            m_setting_.resolution,
            grid_map::Position(m_setting_.x, m_setting_.y));
        if (m_setting_.attached_to_frame) {
            m_grid_map_.setFrameId(m_setting_.attached_frame);
            m_occ_grid_.header.frame_id = m_setting_.attached_frame;
        } else {
            m_grid_map_.setFrameId(m_setting_.world_frame);
            m_occ_grid_.header.frame_id = m_setting_.world_frame;
        }

        // Pre-set OccupancyGrid static fields
        const auto width = static_cast<uint32_t>(m_setting_.x_cells);
        const auto height = static_cast<uint32_t>(m_setting_.y_cells);
        const int half_x = (m_setting_.x_cells - 1) / 2;
        const int half_y = (m_setting_.y_cells - 1) / 2;
        m_occ_grid_.info.resolution = static_cast<float>(m_setting_.resolution);
        m_occ_grid_.info.width = width;
        m_occ_grid_.info.height = height;
        m_occ_grid_.info.origin.position.x =
            m_setting_.x - static_cast<double>(half_x) * m_setting_.resolution;
        m_occ_grid_.info.origin.position.y =
            m_setting_.y - static_cast<double>(half_y) * m_setting_.resolution;
        m_occ_grid_.info.origin.orientation.w = 1.0;
        m_occ_grid_.data.resize(width * height, -1);

        if (m_setting_.publish_rate > 0) {
            m_timer_ = m_nh_.createTimer(
                ros::Duration(1.0 / m_setting_.publish_rate),
                &OccVisualizationNode::CallbackTimer,
                this);
        }
        ROS_INFO("OccVisualizationNode initialized");
    }

private:
    void
    InitQueryPoints() {
        m_query_points_.clear();
        m_query_points_.reserve(m_setting_.x_cells * m_setting_.y_cells);
        int half_x = (m_setting_.x_cells - 1) / 2;
        int half_y = (m_setting_.y_cells - 1) / 2;
        for (int j = half_y; j >= -half_y; --j) {
            for (int i = half_x; i >= -half_x; --i) {
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
    MakeOccQuery(erl_gp_sdf_msgs::OccQuery &srv) {
        if (!m_occ_client_.exists()) {
            ROS_WARN("Service %s not available", m_setting_.occ_query_service.c_str());
            return false;
        }

        srv.request.mode = m_setting_.mode;
        srv.request.compute_gradient = m_setting_.compute_gradient;

        if (m_setting_.attached_to_frame) {
            geometry_msgs::TransformStamped transform_stamped;
            try {
                transform_stamped = m_tf_buffer_.lookupTransform(
                    m_setting_.world_frame,
                    m_setting_.attached_frame,
                    ros::Time(0));
            } catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                return false;
            }
            srv.request.query_points.clear();
            srv.request.query_points.reserve(m_query_points_.size());
            Eigen::Isometry3d transform = tf2::transformToEigen(transform_stamped.transform);
            for (auto &ps: m_query_points_) {
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

        if (!m_occ_client_.call(srv)) {
            ROS_WARN("Failed to call %s", m_occ_client_.getService().c_str());
            return false;
        }

        if (!srv.response.success) {
            ROS_WARN("OccQuery failed: %s", srv.response.reason.c_str());
            return false;
        }

        const auto n = static_cast<int>(srv.response.results.size());
        const auto map_size = m_grid_map_.getSize();
        if (const auto m = map_size[0] * map_size[1]; n != m) {
            ROS_WARN("Query result size %d does not match map size %d", n, m);
            return false;
        }

        return true;
    }

    void
    CallbackTimer(const ros::TimerEvent &) {
        if (m_setting_.publish_grid_map && m_pub_map_.getNumSubscribers() == 0 &&
            m_setting_.publish_occupancy_grid && m_pub_occ_grid_.getNumSubscribers() == 0) {
            return;
        }

        erl_gp_sdf_msgs::OccQuery srv;
        if (!MakeOccQuery(srv)) { return; }
        ProcessResponse(srv.response);
    }

    bool
    ProcessResponse(const erl_gp_sdf_msgs::OccQuery::Response &response) {
        const auto n = static_cast<int>(response.results.size());
        const auto map_size = m_grid_map_.getSize();
        if (const auto m = map_size[0] * map_size[1]; n != m) {
            ROS_WARN("Query result size %d does not match map size %d", n, m);
            return false;
        }

        // Populate GridMap with "occ" layer
        m_grid_map_.setTimestamp(m_stamp_.toNSec());
        m_grid_map_.add(
            "occ",
            Eigen::Map<const Eigen::MatrixXd>(response.results.data(), map_size[0], map_size[1])
                .cast<float>());
        m_grid_map_.setBasicLayers({"occ"});

        // Gradient layers if requested
        if (m_setting_.compute_gradient && !response.gradients.empty()) {
            Eigen::Map<const Eigen::MatrixXd> gradients(
                reinterpret_cast<const double *>(response.gradients.data()),
                3,
                n);
            static const char *gradient_names[3] = {"gradient_x", "gradient_y", "gradient_z"};
            for (int i = 0; i < response.dim; ++i) {
                Eigen::VectorXf grad_i = gradients.row(i).transpose().cast<float>();
                Eigen::Map<Eigen::MatrixXf> grad_map(grad_i.data(), map_size[0], map_size[1]);
                m_grid_map_.add(gradient_names[i], grad_map);
            }
        }

        // Publish GridMap
        if (m_setting_.publish_grid_map) {
            grid_map_msgs::GridMap msg;
            grid_map::GridMapRosConverter::toMessage(m_grid_map_, msg);
            m_pub_map_.publish(msg);
        }

        // Publish OccupancyGrid (only for "prob" mode)
        if (m_setting_.publish_occupancy_grid && m_setting_.mode == "prob") {
            const auto width = m_occ_grid_.info.width;
            const auto height = m_occ_grid_.info.height;

            m_occ_grid_.header.stamp = m_stamp_;
            // response.results: (max_x, max_y) -> (min_x, min_y), x changes first
            // Reversed:         (min_x, min_y) -> (max_x, max_y), x changes first
            // OccupancyGrid:    (min_x, min_y) -> (max_x, max_y), x changes first (row-major)
            const auto total = static_cast<int>(width * height);
            for (int i = 0; i < total; ++i) {
                const auto val = response.results[total - 1 - i];
                if (std::isnan(val)) {
                    m_occ_grid_.data[i] = -1;
                } else {
                    m_occ_grid_.data[i] = static_cast<int8_t>(std::clamp(val * 100.0, 0.0, 100.0));
                }
            }
            m_pub_occ_grid_.publish(m_occ_grid_);
        }

        return true;
    }

    bool
    CallbackSaveQuery(
        erl_gp_sdf_msgs::SaveMap::Request &req,
        erl_gp_sdf_msgs::SaveMap::Response &res) {

        if (req.name.empty()) {
            ROS_WARN("Save query name is empty");
            res.success = false;
            return false;
        }

        erl_gp_sdf_msgs::OccQuery srv;
        if (!MakeOccQuery(srv)) {
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

        // Save occupancy results
        Eigen::Map<const Eigen::VectorXd> occ(ans.results.data(), ans.results.size());
        const auto occ_filepath = dir_path / "occ.bin";
        std::ofstream ofs_occ(occ_filepath, std::ios::binary);
        if (!erl::common::SaveEigenMapToBinaryStream(ofs_occ, occ)) {
            ROS_WARN("Failed to save occ to %s", occ_filepath.c_str());
            res.success = false;
            return false;
        }

        // Save gradients if present
        if (!ans.gradients.empty()) {
            Eigen::Map<const Eigen::MatrixXd> gradients(
                reinterpret_cast<const double *>(ans.gradients.data()),
                ans.dim,
                ans.results.size());
            const auto gradients_filepath = dir_path / "gradients.bin";
            std::ofstream ofs_grad(gradients_filepath, std::ios::binary);
            if (!erl::common::SaveEigenMapToBinaryStream(ofs_grad, gradients)) {
                ROS_WARN("Failed to save gradients to %s", gradients_filepath.c_str());
                res.success = false;
                return false;
            }
        }

        res.success = true;
        return true;
    }

    bool
    CallbackTriggerQuery(std_srvs::Trigger::Request & /*req*/, std_srvs::Trigger::Response &res) {

        erl_gp_sdf_msgs::OccQuery srv;
        if (!MakeOccQuery(srv)) {
            res.success = false;
            res.message = "Failed to create query request";
            return true;
        }

        res.success = ProcessResponse(srv.response);
        res.message = res.success ? "Query and publish completed" : "ProcessResponse failed";
        return true;
    }
};

int
main(int argc, char **argv) {
    ros::init(argc, argv, "occ_visualization_node");
    ros::NodeHandle nh("~");
    OccVisualizationNode node(nh);
    ros::spin();
    return 0;
}
