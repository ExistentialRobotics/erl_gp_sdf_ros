#include "erl_gp_sdf_msgs/srv/sdf_query.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

struct SdfVisualizatioNodeSetting {
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
    bool publish_point_cloud = true;
    // the frequency of publishing the grid map.
    double publish_rate = 2;
    // if true, the grid map is moved with the frame.
    bool attached_to_frame = false;
    // the name of the frame to attach the grid map to.
    std::string attached_frame = "map";
    // the name of the world frame, used when attached_to_frame is true.
    std::string world_frame = "map";
    // the name of the service to query the SDF.
    std::string service_name = "sdf_query";
    // the name of the topic to publish the grid map.
    std::string map_topic_name = "sdf_grid_map";
    // the name of the topic to publish the point cloud.
    std::string point_cloud_topic_name = "sdf_point_cloud";
};

class SdfVisualizatioNode : public rclcpp::Node {
    SdfVisualizatioNodeSetting m_setting_;
    rclcpp::Client<erl_gp_sdf_msgs::srv::SdfQuery>::SharedPtr m_sdf_client_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr m_pub_map_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pub_pcd_;
    rclcpp::TimerBase::SharedPtr m_timer_;
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener_;
    std::vector<geometry_msgs::msg::Vector3> m_query_points_;
    std_msgs::msg::Header m_header_;
    grid_map::GridMap m_grid_map_;
    sensor_msgs::msg::PointCloud2 m_cloud_;
    std::atomic_bool m_last_request_responded_ = true;

public:
    SdfVisualizatioNode()
        : Node("sdf_visualization_node"),
          m_tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
          m_tf_listener_(std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_)) {

        if (!LoadParameters()) {
            RCLCPP_FATAL(this->get_logger(), "Failed to load parameters");
            rclcpp::shutdown();
            return;
        }

        InitQueryPoints();

        m_sdf_client_ =
            this->create_client<erl_gp_sdf_msgs::srv::SdfQuery>(m_setting_.service_name);
        if (m_setting_.publish_grid_map) {
            m_pub_map_ =
                this->create_publisher<grid_map_msgs::msg::GridMap>(m_setting_.map_topic_name, 1);
        }
        if (m_setting_.publish_point_cloud) {
            m_pub_pcd_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                m_setting_.point_cloud_topic_name,
                1);
        }
        if (m_setting_.attached_to_frame) {
            m_grid_map_.setFrameId(m_setting_.attached_frame);
            m_header_.frame_id = m_setting_.attached_frame;
        } else {
            m_grid_map_.setFrameId(m_setting_.world_frame);
            m_header_.frame_id = m_setting_.world_frame;
        }
        m_grid_map_.setGeometry(
            grid_map::Length(
                static_cast<double>(m_setting_.x_cells) * m_setting_.resolution,
                static_cast<double>(m_setting_.y_cells) * m_setting_.resolution),
            m_setting_.resolution,
            grid_map::Position(m_setting_.x, m_setting_.y));
        m_cloud_.header = m_header_;
        m_cloud_.height = 1;
        m_cloud_.is_bigendian = false;
        m_cloud_.is_dense = false;
        m_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&SdfVisualizatioNode::CallbackTimer, this));
        RCLCPP_INFO(this->get_logger(), "SdfVisualizatioNode initialized");
    }

private:
    void
    DeclareParameters() {
        this->declare_parameter("resolution", m_setting_.resolution);
        this->declare_parameter("x_cells", m_setting_.x_cells);
        this->declare_parameter("y_cells", m_setting_.y_cells);
        this->declare_parameter("x", m_setting_.x);
        this->declare_parameter("y", m_setting_.y);
        this->declare_parameter("z", m_setting_.z);
        this->declare_parameter("publish_gradient", m_setting_.publish_gradient);
        this->declare_parameter("publish_sdf_variance", m_setting_.publish_sdf_variance);
        this->declare_parameter("publish_gradient_variance", m_setting_.publish_gradient_variance);
        this->declare_parameter("publish_covariance", m_setting_.publish_covariance);
        this->declare_parameter("publish_grid_map", m_setting_.publish_grid_map);
        this->declare_parameter("publish_point_cloud", m_setting_.publish_point_cloud);
        this->declare_parameter("publish_rate", m_setting_.publish_rate);
        this->declare_parameter("attached_to_frame", m_setting_.attached_to_frame);
        this->declare_parameter("attached_frame", m_setting_.attached_frame);
        this->declare_parameter("service_name", m_setting_.service_name);
        this->declare_parameter("map_topic_name", m_setting_.map_topic_name);
    }

    template<typename T>
    bool
    LoadParam(const std::string& param_name, T& param) {
        try {
            param = this->get_parameter(param_name).template get_value<T>();
        } catch (const rclcpp::exceptions::ParameterNotDeclaredException& e) {
            RCLCPP_FATAL(this->get_logger(), "Failed to load %s", param_name.c_str());
            return false;
        }
        return true;
    }

#define LOAD_PARAM(x) \
    if (!LoadParam(#x, m_setting_.x)) { return false; }

    bool
    LoadParameters() {
        // First declare all parameters
        DeclareParameters();

        // Then load them
        LOAD_PARAM(resolution);
        LOAD_PARAM(x_cells);
        LOAD_PARAM(y_cells);
        LOAD_PARAM(x);
        LOAD_PARAM(y);
        LOAD_PARAM(z);
        LOAD_PARAM(publish_gradient);
        LOAD_PARAM(publish_sdf_variance);
        LOAD_PARAM(publish_gradient_variance);
        LOAD_PARAM(publish_covariance);
        LOAD_PARAM(publish_grid_map);
        LOAD_PARAM(publish_point_cloud);
        LOAD_PARAM(publish_rate);
        LOAD_PARAM(attached_to_frame);
        LOAD_PARAM(attached_frame);
        LOAD_PARAM(service_name);
        LOAD_PARAM(map_topic_name);

        // check the parameters
        if (m_setting_.resolution <= 0) {
            RCLCPP_WARN(this->get_logger(), "Resolution must be positive");
            return false;
        }
        if (m_setting_.x_cells <= 0) {
            RCLCPP_WARN(this->get_logger(), "X cells must be positive");
            return false;
        }
        if (m_setting_.x_cells % 2 == 0) {
            m_setting_.x_cells += 1;
            RCLCPP_WARN(this->get_logger(), "X cells must be odd, set to %d", m_setting_.x_cells);
        }
        if (m_setting_.y_cells <= 0) {
            RCLCPP_WARN(this->get_logger(), "Y cells must be positive");
            return false;
        }
        if (m_setting_.y_cells % 2 == 0) {
            m_setting_.y_cells += 1;
            RCLCPP_WARN(this->get_logger(), "Y cells must be odd, set to %d", m_setting_.y_cells);
        }
        if (!m_setting_.publish_grid_map && !m_setting_.publish_point_cloud) {
            RCLCPP_WARN(
                this->get_logger(),
                "At least one of publish_grid_map or publish_point_cloud must be true");
            return false;
        }
        if (m_setting_.publish_rate <= 0) {
            RCLCPP_WARN(this->get_logger(), "Publish frequency must be positive");
            return false;
        }
        if (m_setting_.attached_to_frame) {
            if (m_setting_.attached_frame.empty()) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Attached frame is empty but attached_to_frame is true");
                return false;
            }
            if (m_setting_.world_frame.empty()) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "World frame is empty but attached_to_frame is true");
                return false;
            }
        }
        if (m_setting_.service_name.empty()) {
            RCLCPP_WARN(this->get_logger(), "Service name is empty");
            return false;
        }
        if (m_setting_.map_topic_name.empty()) {
            RCLCPP_WARN(this->get_logger(), "Map topic name is empty");
            return false;
        }
        return true;
    }

#undef LOAD_PARAM

    void
    InitQueryPoints() {
        m_query_points_.clear();
        m_query_points_.reserve(m_setting_.x_cells * m_setting_.y_cells);
        int half_x = (m_setting_.x_cells - 1) / 2;
        int half_y = (m_setting_.y_cells - 1) / 2;
        for (int j = half_y; j >= -half_y; --j) {      // y-axis
            for (int i = half_x; i >= -half_x; --i) {  // x-axis (column major)
                geometry_msgs::msg::Vector3 p;
                p.x = static_cast<double>(i) * m_setting_.resolution + m_setting_.x;
                p.y = static_cast<double>(j) * m_setting_.resolution + m_setting_.y;
                p.z = m_setting_.z;
                m_query_points_.push_back(p);
            }
        }
        RCLCPP_INFO(
            this->get_logger(),
            "Query points initialized, %d x %d points",
            m_setting_.x_cells,
            m_setting_.y_cells);
    }

    void
    CallbackTimer() {
        if (!m_last_request_responded_.load()) {
            RCLCPP_WARN(this->get_logger(), "Last request is still pending, skipping this cycle");
            return;
        }

        auto request = std::make_shared<erl_gp_sdf_msgs::srv::SdfQuery::Request>();
        if (m_setting_.attached_to_frame) {
            geometry_msgs::msg::TransformStamped transform_stamped;
            try {
                transform_stamped = m_tf_buffer_->lookupTransform(
                    m_setting_.world_frame,
                    m_setting_.attached_frame,
                    tf2::TimePointZero);
            } catch (tf2::TransformException& ex) {
                RCLCPP_WARN(this->get_logger(), ex.what());
                return;
            }
            request->query_points.clear();
            request->query_points.reserve(m_query_points_.size());
            for (auto& ps: m_query_points_) {
                // transform ps from the attached frame to the world frame
                Eigen::Vector4d p_world = tf2::transformToEigen(transform_stamped.transform) *
                                          Eigen::Vector4d(ps.x, ps.y, ps.z, 1.0);
                geometry_msgs::msg::Vector3 p;
                p.x = p_world.x();
                p.y = p_world.y();
                p.z = p_world.z();
                request->query_points.emplace_back(std::move(p));
            }
            m_header_.stamp = transform_stamped.header.stamp;
        } else {
            request->query_points = m_query_points_;
            m_header_.stamp = this->now();
        }

        if (!m_sdf_client_->service_is_ready()) {
            RCLCPP_WARN(
                this->get_logger(),
                "Service %s is not ready",
                m_sdf_client_->get_service_name());
            return;
        }

        auto callback =
            [this](rclcpp::Client<erl_gp_sdf_msgs::srv::SdfQuery>::SharedFutureWithRequest future) {
                this->ResponseHandler(std::move(future));
            };
        m_last_request_responded_.store(false);
        auto result_future = m_sdf_client_->async_send_request(request, callback);
    }

    void
    ResponseHandler(
        rclcpp::Client<erl_gp_sdf_msgs::srv::SdfQuery>::SharedFutureWithRequest future) {
        m_last_request_responded_.store(true);
        auto request_response_pair = future.get();
        auto& response = request_response_pair.second;
        if (!response->success) {
            RCLCPP_WARN(this->get_logger(), "SDF query failed");
            return;
        }

        const auto n = static_cast<int>(response->signed_distances.size());
        const auto map_size = m_grid_map_.getSize();
        if (n != map_size[0] * map_size[1]) {
            RCLCPP_WARN(
                this->get_logger(),
                "Query points size %d does not match map size %d",
                n,
                map_size[0] * map_size[1]);
            return;
        }

        // 1. copy the result to the grid map / the point cloud
        // 2. publish the grid map / point cloud

        if (m_setting_.publish_grid_map) {
            m_grid_map_.setTimestamp(rclcpp::Time(m_header_.stamp).nanoseconds());
            LoadResponseToGridMap(*response);
            auto msg = grid_map::GridMapRosConverter::toMessage(m_grid_map_);
            m_pub_map_->publish(std::move(msg));
        }

        if (m_setting_.publish_point_cloud) {
            m_cloud_.header = m_header_;
            m_cloud_.data.clear();
            InitializePointCloudFields(*response);
            LoadResponseToPointCloud(*response);
            m_pub_pcd_->publish(m_cloud_);
        }
    }

    void
    InitializePointCloudFields(const erl_gp_sdf_msgs::srv::SdfQuery::Response& ans) {
        if (!m_cloud_.fields.empty()) { return; }
        // initialize point cloud fields
        m_cloud_.fields.reserve(14);
        sensor_msgs::msg::PointField field;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;

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
                RCLCPP_WARN(this->get_logger(), "Unknown dimension %d", ans.dim);
                return;
            }
        }
        m_cloud_.point_step = field.offset + 4;
    }

    void
    LoadResponseToGridMap(const erl_gp_sdf_msgs::srv::SdfQuery::Response& ans) {
        if (!m_setting_.publish_grid_map) { return; }
        const auto map_size = m_grid_map_.getSize();
        const auto n = static_cast<int>(ans.signed_distances.size());
        if (n != map_size[0] * map_size[1]) {
            RCLCPP_WARN(
                this->get_logger(),
                "Query points size %d does not match map size %d",
                n,
                map_size[0] * map_size[1]);
            return;
        }

        // SDF
        m_grid_map_.add(
            "sdf",
            Eigen::Map<const Eigen::MatrixXd>(ans.signed_distances.data(), map_size[0], map_size[1])
                .cast<float>());
        m_grid_map_.setBasicLayers({"sdf"});
        // gradient
        if (m_setting_.publish_gradient) {  // dim layers
            if (ans.compute_gradient) {
                Eigen::Map<const Eigen::MatrixXd> gradients(
                    reinterpret_cast<const double*>(ans.gradients.data()),
                    3,
                    n);
                static const char* gradient_names[3] = {"gradient_x", "gradient_y", "gradient_z"};
                for (int i = 0; i < ans.dim; ++i) {
                    Eigen::VectorXf grad_i = gradients.row(i).transpose().cast<float>();
                    Eigen::Map<Eigen::MatrixXf> grad_map(grad_i.data(), map_size[0], map_size[1]);
                    m_grid_map_.add(gradient_names[i], grad_map);
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "Gradient is not computed");
            }
        }
        Eigen::Map<const Eigen::MatrixXd> variances(
            reinterpret_cast<const double*>(ans.variances.data()),
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
                static const char* gradient_variance_names[3] = {
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
                RCLCPP_WARN(this->get_logger(), "Gradient variance is not computed");
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
                    static const char* covariance_names[3] = {
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
                    static const char* covariance_names[6] = {
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
                    RCLCPP_WARN(this->get_logger(), "Unknown dimension %d", ans.dim);
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "Covariance is not computed");
            }
        }
    }

    void
    LoadResponseToPointCloud(const erl_gp_sdf_msgs::srv::SdfQuery::Response& ans) {
        if (!m_setting_.publish_point_cloud) { return; }
        const auto map_size = m_grid_map_.getSize();
        const auto n = static_cast<int>(ans.signed_distances.size());
        if (n != map_size[0] * map_size[1]) {
            RCLCPP_WARN(
                this->get_logger(),
                "Query points size %d does not match map size %d",
                n,
                map_size[0] * map_size[1]);
            return;
        }

        m_cloud_.data.resize(m_cloud_.point_step * n);
        const double* dummy_ptr = ans.signed_distances.data();
        Eigen::Map<Eigen::MatrixXf> data_out(
            reinterpret_cast<float*>(m_cloud_.data.data()),
            m_cloud_.point_step / 4,
            n);
        Eigen::Map<const Eigen::MatrixXd> gradients(
            m_setting_.publish_gradient && ans.compute_gradient
                ? reinterpret_cast<const double*>(ans.gradients.data())
                : dummy_ptr,
            3,
            n);
        Eigen::Map<const Eigen::MatrixXd> variances(
            reinterpret_cast<const double*>(ans.variances.data()),
            ans.compute_gradient_variance ? ans.dim + 1 : 1,
            n);
        Eigen::Map<const Eigen::MatrixXd> covariances(
            ans.compute_covariance ? ans.covariances.data() : dummy_ptr,
            ans.dim * (ans.dim + 1) / 2,
            n);

        int j = 0;  // index for the point cloud data
        for (int i = 0; i < n; ++i) {
            const double* var_ptr = variances.col(i).data();
            if (var_ptr[0] >= 1.0e5) { continue; }  // invalid SDF value, skip this point
            float* out = data_out.col(j++).data();
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
                const double* cov_ptr = covariances.col(i).data();
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
                    RCLCPP_WARN(this->get_logger(), "Unknown dimension %d", ans.dim);
                }
            }
        }

        m_cloud_.width = j;
        m_cloud_.row_step = m_cloud_.point_step * j;
        m_cloud_.data.resize(m_cloud_.row_step);
        for (auto& field: m_cloud_.fields) { field.count = j; }
    }
};

int
main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SdfVisualizatioNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
