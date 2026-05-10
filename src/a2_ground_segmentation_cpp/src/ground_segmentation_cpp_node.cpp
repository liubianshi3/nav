// Copyright 2026 a2_system_ws.
// ROS 2 wrapper around `a2_ground_segmentation_cpp::GroundSegmenter`.
// Drop-in replacement for the Python `a2_ground_segmentation` node.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/string.hpp>

#include "a2_ground_segmentation_cpp/ground_segmenter.hpp"

namespace a2gs = a2_ground_segmentation_cpp;

namespace
{

sensor_msgs::msg::PointCloud2 build_xyz_cloud(
  const std_msgs::msg::Header & header,
  const std::string & frame_id_override,
  const Eigen::Ref<const Eigen::MatrixX3f> & xyz)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header = header;
  if (!frame_id_override.empty()) {
    msg.header.frame_id = frame_id_override;
  }
  const auto N = static_cast<uint32_t>(xyz.rows());
  msg.height = 1;
  msg.width = N;
  msg.is_bigendian = false;
  msg.is_dense = true;
  msg.point_step = 12;
  msg.row_step = msg.point_step * N;
  msg.fields.resize(3);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[1].count = 1;
  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[2].count = 1;
  msg.data.resize(static_cast<size_t>(msg.row_step));
  if (N > 0) {
    // xyz is row-major (Nx3 floats), exactly 12 bytes per row.
    std::memcpy(msg.data.data(), xyz.data(), static_cast<size_t>(N) * 12u);
  }
  return msg;
}

}  // namespace

class GroundSegmentationCppNode : public rclcpp::Node
{
public:
  GroundSegmentationCppNode()
  : rclcpp::Node("ground_segmentation_cpp")
  {
    declare_and_load_params();
    try {
      segmenter_ = std::make_unique<a2gs::GroundSegmenter>(params_);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(get_logger(), "Invalid parameters at startup: %s", e.what());
      throw;
    }

    // Publishers
    ground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(ground_topic_, 10);
    obstacle_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(obstacle_topic_, 10);
    traversability_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(traversability_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);

    // Subscriber: best-effort + KEEP_LAST(1) to drop stale frames at high rates.
    rclcpp::QoS sub_qos(rclcpp::KeepLast(1));
    sub_qos.best_effort();
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, sub_qos,
      std::bind(&GroundSegmentationCppNode::on_cloud, this, std::placeholders::_1));

    using namespace std::chrono_literals;
    const auto period = std::chrono::duration<double>(
      1.0 / std::max(0.1, params_.traversability_publish_hz));
    traversability_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&GroundSegmentationCppNode::on_traversability_timer, this));
    status_timer_ = create_wall_timer(
      1s, std::bind(&GroundSegmentationCppNode::publish_status, this));

    RCLCPP_INFO(
      get_logger(),
      "ground_segmentation_cpp ready: input=%s ground=%s obstacle=%s "
      "trav=%s sectors=%d general=%.1fdeg local=%.1fdeg",
      input_topic_.c_str(), ground_topic_.c_str(),
      obstacle_topic_.c_str(), traversability_topic_.c_str(),
      static_cast<int>(std::ceil(360.0 / std::max(0.1, params_.radial_divider_angle_deg))),
      params_.general_max_slope_deg, params_.local_max_slope_deg);
  }

private:
  void declare_and_load_params()
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/jt128/front/points");
    ground_topic_ = declare_parameter<std::string>("ground_topic", "/a2/ground/points");
    obstacle_topic_ = declare_parameter<std::string>("obstacle_topic", "/a2/obstacle/points");
    traversability_topic_ = declare_parameter<std::string>(
      "traversability_topic", "/a2/traversability");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/a2/perception/ground_segmentation/status");

    params_.radial_divider_angle_deg = declare_parameter<double>("radial_divider_angle", 1.0);
    params_.general_max_slope_deg = declare_parameter<double>("general_max_slope_deg", 8.0);
    params_.local_max_slope_deg = declare_parameter<double>("local_max_slope_deg", 6.0);
    params_.initial_max_slope_deg = declare_parameter<double>("initial_max_slope_deg", 3.0);
    params_.min_height_threshold = declare_parameter<double>("min_height_threshold", 0.15);
    params_.concentric_divider_distance = declare_parameter<double>(
      "concentric_divider_distance", 0.01);
    params_.reclass_distance_threshold = declare_parameter<double>(
      "reclass_distance_threshold", 0.1);

    params_.traversability_resolution = declare_parameter<double>("traversability_resolution", 0.1);
    params_.traversability_width = static_cast<int>(declare_parameter<int>("traversability_width", 400));
    params_.traversability_height = static_cast<int>(declare_parameter<int>("traversability_height", 400));
    params_.traversability_origin_x = declare_parameter<double>("traversability_origin_x", -20.0);
    params_.traversability_origin_y = declare_parameter<double>("traversability_origin_y", -20.0);
    params_.max_traversable_slope_deg = declare_parameter<double>(
      "max_traversable_slope_deg", 20.0);
    params_.traversability_publish_hz = declare_parameter<double>(
      "traversability_publish_hz", 1.0);
    params_.traversability_height_ema_alpha = declare_parameter<double>(
      "traversability_height_ema_alpha", 0.3);
    params_.traversability_min_count_known = static_cast<int>(declare_parameter<int>(
      "traversability_min_count_known", 3));

    frame_id_override_ = declare_parameter<std::string>("frame_id", "map");
    process_every_n_ = std::max<int>(1, static_cast<int>(declare_parameter<int>("process_every_n", 1)));
    max_consecutive_failures_ = std::max<int>(
      1, static_cast<int>(declare_parameter<int>("max_consecutive_failures", 5)));
  }

  Eigen::MatrixX3f extract_xyz(const sensor_msgs::msg::PointCloud2 & msg) const
  {
    const std::size_t n = static_cast<std::size_t>(msg.width) * msg.height;
    Eigen::MatrixX3f out(static_cast<Eigen::Index>(n), 3);
    if (n == 0) {
      return Eigen::MatrixX3f(0, 3);
    }
    sensor_msgs::PointCloud2ConstIterator<float> ix(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(msg, "z");
    Eigen::Index w = 0;
    for (std::size_t i = 0; i < n; ++i, ++ix, ++iy, ++iz) {
      const float x = *ix;
      const float y = *iy;
      const float z = *iz;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      out(w, 0) = x;
      out(w, 1) = y;
      out(w, 2) = z;
      ++w;
    }
    out.conservativeResize(w, 3);
    return out;
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    ++recv_count_;
    if ((recv_count_ % static_cast<std::uint64_t>(process_every_n_)) != 0) {
      ++skipped_frames_;
      return;
    }
    const auto t0 = std::chrono::steady_clock::now();
    try {
      Eigen::MatrixX3f xyz = extract_xyz(*msg);
      const auto N = static_cast<std::size_t>(xyz.rows());
      if (N < 10) {
        ++empty_frames_;
        consecutive_failures_ = 0;
        return;
      }
      auto cls = segmenter_->classify(xyz);

      // Split ground / obstacle, preserving original order.
      Eigen::MatrixX3f ground(static_cast<Eigen::Index>(cls.ground_count), 3);
      Eigen::MatrixX3f obstacle(
        static_cast<Eigen::Index>(N - cls.ground_count), 3);
      Eigen::Index gi = 0;
      Eigen::Index oi = 0;
      for (std::size_t i = 0; i < N; ++i) {
        const auto row = static_cast<Eigen::Index>(i);
        if (cls.ground_mask[i]) {
          ground.row(gi++) = xyz.row(row);
        } else {
          obstacle.row(oi++) = xyz.row(row);
        }
      }

      ground_pub_->publish(build_xyz_cloud(msg->header, frame_id_override_, ground));
      obstacle_pub_->publish(build_xyz_cloud(msg->header, frame_id_override_, obstacle));
      segmenter_->accumulate_traversability(ground);

      const auto t1 = std::chrono::steady_clock::now();
      const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
      // EWMA of latency for status reporting
      latency_ms_ema_ = (latency_ms_ema_ <= 0.0) ? ms : (0.9 * latency_ms_ema_ + 0.1 * ms);
      ++processed_frames_;
      consecutive_failures_ = 0;
    } catch (const std::exception & e) {
      ++exception_frames_;
      ++consecutive_failures_;
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ground_segmentation_cpp on_cloud exception: %s", e.what());
    } catch (...) {
      ++exception_frames_;
      ++consecutive_failures_;
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ground_segmentation_cpp on_cloud unknown exception");
    }
  }

  void on_traversability_timer()
  {
    try {
      std::vector<int8_t> data;
      if (!segmenter_->render_traversability(data)) {
        return;
      }
      nav_msgs::msg::OccupancyGrid grid;
      grid.header.stamp = now();
      grid.header.frame_id = frame_id_override_;
      grid.info.resolution = static_cast<float>(params_.traversability_resolution);
      grid.info.width = static_cast<uint32_t>(params_.traversability_width);
      grid.info.height = static_cast<uint32_t>(params_.traversability_height);
      grid.info.origin.position.x = params_.traversability_origin_x;
      grid.info.origin.position.y = params_.traversability_origin_y;
      grid.info.origin.position.z = 0.0;
      grid.info.origin.orientation.w = 1.0;
      grid.data = std::move(data);
      traversability_pub_->publish(grid);
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ground_segmentation_cpp traversability publish error: %s", e.what());
    }
  }

  void publish_status()
  {
    std_msgs::msg::String msg;
    const bool error = consecutive_failures_ >= max_consecutive_failures_;
    const char * state = error ? "error" : (processed_frames_ > 0 ? "ready" : "waiting_cloud");
    char buf[512];
    std::snprintf(
      buf, sizeof(buf),
      "mode=cpp;state=%s;ready=%s;reason=%s;processed=%lu;skipped=%lu;empty=%lu;"
      "exceptions=%lu;latency_ms_ewma=%.2f;known_cells=%lu",
      state,
      error ? "false" : (processed_frames_ > 0 ? "true" : "false"),
      error ? "consecutive_segmentation_failures" :
        (processed_frames_ > 0 ? "ok" : "no_cloud_yet"),
      static_cast<unsigned long>(processed_frames_),
      static_cast<unsigned long>(skipped_frames_),
      static_cast<unsigned long>(empty_frames_),
      static_cast<unsigned long>(exception_frames_),
      latency_ms_ema_,
      static_cast<unsigned long>(segmenter_ ? segmenter_->known_cell_count() : 0));
    msg.data = buf;
    status_pub_->publish(msg);
  }

  // Params
  a2gs::GroundSegmenterParams params_;
  std::string input_topic_;
  std::string ground_topic_;
  std::string obstacle_topic_;
  std::string traversability_topic_;
  std::string status_topic_;
  std::string frame_id_override_;
  int process_every_n_{1};
  int max_consecutive_failures_{5};

  // State
  std::unique_ptr<a2gs::GroundSegmenter> segmenter_;
  std::uint64_t recv_count_{0};
  std::uint64_t processed_frames_{0};
  std::uint64_t skipped_frames_{0};
  std::uint64_t empty_frames_{0};
  std::uint64_t exception_frames_{0};
  int consecutive_failures_{0};
  double latency_ms_ema_{-1.0};

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr traversability_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr traversability_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<GroundSegmentationCppNode>());
  } catch (const std::exception & e) {
    fprintf(stderr, "ground_segmentation_cpp_node fatal: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
