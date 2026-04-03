#include "erl_common/eigen.hpp"
#include "erl_common/ros2_topic_params.hpp"
#include "erl_gp_sdf_msgs/srv/occ_query.hpp"
#include "erl_gp_sdf_msgs/srv/save_map.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>

using namespace erl::common;
using namespace erl::common::ros_params;

static rclcpp::Node *g_curr_node = nullptr;

struct OccVisNodeConfig : public Yamlable<OccVisNodeConfig> {
    double resolution = 0.1;
    long x_cells = 101;
    long y_cells = 101;
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
    Ros2TopicParams occ_query_service{"occ_query", "services"};
    Ros2TopicParams save_query_service{"occ_save_query", "services"};
    Ros2TopicParams trigger_query_service{"occ_trigger_query", "services"};
    Ros2TopicParams map_topic{"occ_grid_map"};
    Ros2TopicParams occupancy_grid_topic{"occupancy_grid"};

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
        auto logger = g_curr_node->get_logger();
        if (resolution <= 0) {
            RCLCPP_WARN(logger, "Resolution must be positive");
            return false;
        }
        if (x_cells <= 0) {
            RCLCPP_WARN(logger, "X cells must be positive");
            return false;
        }
        if (x_cells % 2 == 0) {
            x_cells += 1;
            RCLCPP_WARN(logger, "X cells must be odd, set to %ld", x_cells);
        }
        if (y_cells <= 0) {
            RCLCPP_WARN(logger, "Y cells must be positive");
            return false;
        }
        if (y_cells % 2 == 0) {
            y_cells += 1;
            RCLCPP_WARN(logger, "Y cells must be odd, set to %ld", y_cells);
        }
        if (mode != "logodd" && mode != "prob") {
            RCLCPP_WARN(logger, "mode must be 'logodd' or 'prob', got '%s'", mode.c_str());
            return false;
        }
        if (!publish_grid_map && !publish_occupancy_grid) {
            RCLCPP_WARN(
                logger,
                "At least one of publish_grid_map or publish_occupancy_grid must be true");
            return false;
        }
        if (attached_to_frame) {
            if (attached_frame.empty()) {
                RCLCPP_WARN(logger, "Attached frame is empty but attached_to_frame is true");
                return false;
            }
            if (world_frame.empty()) {
                RCLCPP_WARN(logger, "World frame is empty but attached_to_frame is true");
                return false;
            }
        }
        if (occ_query_service.path.empty()) {
            RCLCPP_WARN(logger, "occ_query_service.path is empty");
            return false;
        }
        if (save_query_service.path.empty()) {
            RCLCPP_WARN(logger, "save_query_service.path is empty");
            return false;
        }
        if (trigger_query_service.path.empty()) {
            RCLCPP_WARN(logger, "trigger_query_service.path is empty");
            return false;
        }
        if (map_topic.path.empty()) {
            RCLCPP_WARN(logger, "Map topic path is empty");
            return false;
        }
        if (occupancy_grid_topic.path.empty()) {
            RCLCPP_WARN(logger, "Occupancy grid topic path is empty");
            return false;
        }
        return true;
    }
};

class OccVisualizationNode : public rclcpp::Node {
    OccVisNodeConfig m_setting_;
    rclcpp::Client<erl_gp_sdf_msgs::srv::OccQuery>::SharedPtr m_occ_client_;
    rclcpp::Service<erl_gp_sdf_msgs::srv::SaveMap>::SharedPtr m_srv_save_query_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_srv_trigger_query_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr m_pub_map_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_pub_occ_grid_;
    rclcpp::TimerBase::SharedPtr m_timer_;
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener_;
    std::vector<geometry_msgs::msg::Vector3> m_query_points_;
    rclcpp::Time m_stamp_;
    grid_map::GridMap m_grid_map_;
    nav_msgs::msg::OccupancyGrid m_occ_grid_;
    std::atomic_bool m_last_request_responded_ = true;

public:
    OccVisualizationNode()
        : Node("occ_visualization_node"),
          m_tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
          m_tf_listener_(std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_)) {

        g_curr_node = this;

        auto logger = this->get_logger();
        if (!m_setting_.LoadFromRos2(this, "")) {
            RCLCPP_FATAL(logger, "Failed to load parameters");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(logger, "Loaded node parameters:\n%s", m_setting_.AsYamlString().c_str());

        InitQueryPoints();
#ifdef ROS_HUMBLE
        m_occ_client_ = this->create_client<erl_gp_sdf_msgs::srv::OccQuery>(
            m_setting_.occ_query_service.path,
            m_setting_.occ_query_service.GetQoS().get_rmw_qos_profile());
        m_srv_save_query_ = this->create_service<erl_gp_sdf_msgs::srv::SaveMap>(
            m_setting_.save_query_service.path,
            std::bind(
                &OccVisualizationNode::CallbackSaveQuery,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            m_setting_.save_query_service.GetQoS().get_rmw_qos_profile());
        m_srv_trigger_query_ = this->create_service<std_srvs::srv::Trigger>(
            m_setting_.trigger_query_service.path,
            std::bind(
                &OccVisualizationNode::CallbackTriggerQuery,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            m_setting_.trigger_query_service.GetQoS().get_rmw_qos_profile());
#else
        m_occ_client_ = this->create_client<erl_gp_sdf_msgs::srv::OccQuery>(
            m_setting_.occ_query_service.path,
            m_setting_.occ_query_service.GetQoS());
        m_srv_save_query_ = this->create_service<erl_gp_sdf_msgs::srv::SaveMap>(
            m_setting_.save_query_service.path,
            std::bind(
                &OccVisualizationNode::CallbackSaveQuery,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            m_setting_.save_query_service.GetQoS());
        m_srv_trigger_query_ = this->create_service<std_srvs::srv::Trigger>(
            m_setting_.trigger_query_service.path,
            std::bind(
                &OccVisualizationNode::CallbackTriggerQuery,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            m_setting_.trigger_query_service.GetQoS());
#endif
        if (m_setting_.publish_grid_map) {
            m_pub_map_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
                m_setting_.map_topic.path,
                m_setting_.map_topic.GetQoS());
        }
        if (m_setting_.publish_occupancy_grid) {
            m_pub_occ_grid_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
                m_setting_.occupancy_grid_topic.path,
                m_setting_.occupancy_grid_topic.GetQoS());
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
        const long half_x = (m_setting_.x_cells - 1) / 2;
        const long half_y = (m_setting_.y_cells - 1) / 2;
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
            m_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(1000.0 / m_setting_.publish_rate)),
                std::bind(&OccVisualizationNode::CallbackTimer, this));
        }
        RCLCPP_INFO(this->get_logger(), "OccVisualizationNode initialized");
    }

private:
    void
    InitQueryPoints() {
        m_query_points_.clear();
        m_query_points_.reserve(m_setting_.x_cells * m_setting_.y_cells);
        long half_x = (m_setting_.x_cells - 1) / 2;
        long half_y = (m_setting_.y_cells - 1) / 2;
        for (long j = half_y; j >= -half_y; --j) {
            for (long i = half_x; i >= -half_x; --i) {
                geometry_msgs::msg::Vector3 p;
                p.x = static_cast<double>(i) * m_setting_.resolution + m_setting_.x;
                p.y = static_cast<double>(j) * m_setting_.resolution + m_setting_.y;
                p.z = m_setting_.z;
                m_query_points_.push_back(p);
            }
        }
        RCLCPP_INFO(
            this->get_logger(),
            "Query points initialized, %ld x %ld points",
            m_setting_.x_cells,
            m_setting_.y_cells);
    }

    erl_gp_sdf_msgs::srv::OccQuery::Request::SharedPtr
    MakeOccQueryRequest() {
        auto logger = this->get_logger();

        if (!m_occ_client_->service_is_ready()) {
            RCLCPP_WARN(logger, "Service %s is not ready", m_occ_client_->get_service_name());
            return nullptr;
        }

        if (!m_last_request_responded_.load()) {
            RCLCPP_WARN_THROTTLE(logger, *this->get_clock(),  10000.0, "Last request is still pending, skipping this cycle");
            return nullptr;
        }

        auto request = std::make_shared<erl_gp_sdf_msgs::srv::OccQuery::Request>();
        request->mode = m_setting_.mode;
        request->compute_gradient = m_setting_.compute_gradient;

        if (m_setting_.attached_to_frame) {
            geometry_msgs::msg::TransformStamped transform_stamped;
            try {
                transform_stamped = m_tf_buffer_->lookupTransform(
                    m_setting_.world_frame,
                    m_setting_.attached_frame,
                    tf2::TimePointZero);
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(logger, ex.what());
                return nullptr;
            }
            request->query_points.clear();
            request->query_points.reserve(m_query_points_.size());
            Eigen::Isometry3d transform = tf2::transformToEigen(transform_stamped.transform);
            for (auto &ps: m_query_points_) {
                Eigen::Vector4d p_world = transform * Eigen::Vector4d(ps.x, ps.y, ps.z, 1.0);
                geometry_msgs::msg::Vector3 p;
                p.x = p_world.x();
                p.y = p_world.y();
                p.z = p_world.z();
                request->query_points.emplace_back(std::move(p));
            }
            m_stamp_ = transform_stamped.header.stamp;
        } else {
            request->query_points = m_query_points_;
            m_stamp_ = this->now();
        }

        return request;
    }

    bool
    HasSubscribers() const {
        if (m_pub_map_ && m_pub_map_->get_subscription_count() > 0) { return true; }
        if (m_pub_occ_grid_ && m_pub_occ_grid_->get_subscription_count() > 0) { return true; }
        return false;
    }

    void
    CallbackTimer() {
        if (!HasSubscribers()) { return; }
        auto request = MakeOccQueryRequest();
        if (!request) { return; }
        m_last_request_responded_.store(false);
        auto callback =
            [this](rclcpp::Client<erl_gp_sdf_msgs::srv::OccQuery>::SharedFutureWithRequest future) {
                this->ResponseHandler(std::move(future));
            };
        m_occ_client_->async_send_request(request, callback);
    }

    void
    ResponseHandler(
        rclcpp::Client<erl_gp_sdf_msgs::srv::OccQuery>::SharedFutureWithRequest future) {
        m_last_request_responded_.store(true);
        auto request_response_pair = future.get();
        ProcessResponse(*request_response_pair.second);
    }

    /// Populate grid map from query response and publish. Returns true on success.
    bool
    ProcessResponse(const erl_gp_sdf_msgs::srv::OccQuery::Response &response) {
        auto logger = this->get_logger();

        if (!response.success) {
            RCLCPP_WARN(logger, "OccQuery failed: %s", response.reason.c_str());
            return false;
        }

        const auto n = static_cast<int>(response.results.size());
        const auto map_size = m_grid_map_.getSize();
        if (const auto m = map_size[0] * map_size[1]; n != m) {
            RCLCPP_WARN(logger, "Query result size %d does not match map size %d", n, m);
            return false;
        }

        // Populate GridMap with "occ" layer
        m_grid_map_.setTimestamp(m_stamp_.nanoseconds());
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
            auto msg = grid_map::GridMapRosConverter::toMessage(m_grid_map_);
            m_pub_map_->publish(std::move(msg));
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
                    m_occ_grid_.data[i] = static_cast<int8_t>(
                        std::clamp(val * 100.0, 0.0, 100.0));
                }
            }
            m_pub_occ_grid_->publish(m_occ_grid_);
        }

        return true;
    }

    void
    CallbackSaveQuery(
        erl_gp_sdf_msgs::srv::SaveMap::Request::ConstSharedPtr req,
        erl_gp_sdf_msgs::srv::SaveMap::Response::SharedPtr res) {

        if (req->name.empty()) {
            RCLCPP_WARN(this->get_logger(), "Save query name is empty");
            res->success = false;
            return;
        }

        auto occ_request = MakeOccQueryRequest();
        if (!occ_request) {
            res->success = false;
            return;
        }
        auto future = m_occ_client_->async_send_request(occ_request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_WARN(
                this->get_logger(),
                "Failed to call service %s",
                m_occ_client_->get_service_name());
            res->success = false;
            return;
        }

        auto occ_response = future.get();
        if (!occ_response->success) {
            RCLCPP_WARN(this->get_logger(), "OccQuery failed: %s", occ_response->reason.c_str());
            res->success = false;
            return;
        }

        if (!std::filesystem::exists(req->name)) { std::filesystem::create_directories(req->name); }
        if (!std::filesystem::is_directory(req->name)) {
            RCLCPP_WARN(
                this->get_logger(),
                "Save query path %s is not a directory",
                req->name.c_str());
            res->success = false;
            return;
        }

        auto &ans = *occ_response;
        std::filesystem::path dir_path(req->name);

        // Save occupancy results
        Eigen::Map<const Eigen::VectorXd> occ(ans.results.data(), ans.results.size());
        const auto occ_filepath = dir_path / "occ.bin";
        std::ofstream ofs_occ(occ_filepath, std::ios::binary);
        if (!erl::common::SaveEigenMapToBinaryStream(ofs_occ, occ)) {
            RCLCPP_WARN(this->get_logger(), "Failed to save occ to %s", occ_filepath.c_str());
            res->success = false;
            return;
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
                RCLCPP_WARN(
                    this->get_logger(),
                    "Failed to save gradients to %s",
                    gradients_filepath.c_str());
                res->success = false;
                return;
            }
        }

        res->success = true;
    }

    void
    CallbackTriggerQuery(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {

        auto occ_request = MakeOccQueryRequest();
        if (!occ_request) {
            res->success = false;
            res->message = "Failed to create query request";
            return;
        }
        auto future = m_occ_client_->async_send_request(occ_request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            res->success = false;
            res->message = "Failed to call OccQuery service";
            return;
        }

        res->success = ProcessResponse(*future.get());
        res->message = res->success ? "Query and publish completed" : "ProcessResponse failed";
    }
};

int
main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OccVisualizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
