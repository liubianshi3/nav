// Copyright 2026 a2_system_ws.
// ROS 2 wrapper around `a2_ground_segmentation_cpp::GroundSegmenter`.
// Drop-in replacement for the Python `a2_ground_segmentation` node.
//
// V2: Multi-layer cell statistics, time decay, graded traversability cost,
// debug publishers for slope/roughness/step/obstacle_density/confidence/reason.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "a2_ground_segmentation_cpp/ground_segmenter.hpp"

namespace a2gs = a2_ground_segmentation_cpp;

namespace
{

sensor_msgs::msg::PointCloud2 build_xyz_cloud(
  const std_msgs::msg::Header & header,
  const Eigen::Ref<const Eigen::MatrixX3f> & xyz)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header = header;
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
    sensor_msgs::PointCloud2Iterator<float> ox(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> oy(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> oz(msg, "z");
    for (Eigen::Index i = 0; i < xyz.rows(); ++i, ++ox, ++oy, ++oz) {
      *ox = xyz(i, 0);
      *oy = xyz(i, 1);
      *oz = xyz(i, 2);
    }
  }
  return msg;
}

Eigen::Isometry3f transform_to_isometry(const geometry_msgs::msg::TransformStamped & transform)
{
  Eigen::Quaternionf q(
    static_cast<float>(transform.transform.rotation.w),
    static_cast<float>(transform.transform.rotation.x),
    static_cast<float>(transform.transform.rotation.y),
    static_cast<float>(transform.transform.rotation.z));
  if (q.norm() == 0.0f) {
    q = Eigen::Quaternionf::Identity();
  } else {
    q.normalize();
  }

  Eigen::Isometry3f iso = Eigen::Isometry3f::Identity();
  iso.linear() = q.toRotationMatrix();
  iso.translation() = Eigen::Vector3f(
    static_cast<float>(transform.transform.translation.x),
    static_cast<float>(transform.transform.translation.y),
    static_cast<float>(transform.transform.translation.z));
  return iso;
}

nav_msgs::msg::OccupancyGrid make_grid_header(
  const std::string & frame_id,
  double resolution,
  int width,
  int height,
  double origin_x,
  double origin_y,
  rclcpp::Time stamp)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.stamp = stamp;
  grid.header.frame_id = frame_id;
  grid.info.resolution = static_cast<float>(resolution);
  grid.info.width = static_cast<uint32_t>(width);
  grid.info.height = static_cast<uint32_t>(height);
  grid.info.origin.position.x = origin_x;
  grid.info.origin.position.y = origin_y;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;
  return grid;
}

struct FilteredCloud
{
  Eigen::MatrixX3f classification_xyz;
  Eigen::MatrixX3f target_xyz;
};

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

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Publishers
    ground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(ground_topic_, 10);
    obstacle_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(obstacle_topic_, 10);
    traversability_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(traversability_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);

    // V2 debug publishers
    debug_slope_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      debug_topic_prefix_ + "/slope", 10);
    debug_roughness_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      debug_topic_prefix_ + "/roughness", 10);
    debug_step_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      debug_topic_prefix_ + "/step", 10);
    debug_obstacle_density_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      debug_topic_prefix_ + "/obstacle_density", 10);
    debug_confidence_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      debug_topic_prefix_ + "/confidence", 10);
    debug_reason_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      debug_topic_prefix_ + "/reason", 10);

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
      "ground_segmentation_cpp ready: input=%s target_frame=%s self_filter=%s "
      "classification_frame=%s ground=%s obstacle=%s trav=%s sectors=%d "
      "general=%.1fdeg local=%.1fdeg classification_z_offset=%.2fm "
      "ground_plane=%s v2=%s",
      input_topic_.c_str(), target_frame_.c_str(),
      self_filter_enabled_ ? "true" : "false", classification_frame_.c_str(),
      ground_topic_.c_str(), obstacle_topic_.c_str(), traversability_topic_.c_str(),
      static_cast<int>(std::ceil(360.0 / std::max(0.1, params_.radial_divider_angle_deg))),
      params_.general_max_slope_deg, params_.local_max_slope_deg,
      classification_z_offset_m_,
      classification_ground_plane_enabled_ ? "true" : "false",
      params_.traversability_v2_enabled ? "true" : "false");
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
    debug_topic_prefix_ = declare_parameter<std::string>(
      "debug_topic_prefix", "/a2/traversability/debug");

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

    // ── V2 parameters ─────────────────────────────────────────────────
    params_.traversability_v2_enabled = declare_parameter<bool>(
      "traversability_v2_enabled", true);
    params_.traversability_cell_timeout_sec = declare_parameter<double>(
      "traversability_cell_timeout_sec", 1.5);
    params_.traversability_confidence_decay_per_sec = declare_parameter<double>(
      "traversability_confidence_decay_per_sec", 0.8);
    params_.traversability_min_confidence = declare_parameter<double>(
      "traversability_min_confidence", 0.35);
    params_.traversability_unknown_policy = declare_parameter<std::string>(
      "traversability_unknown_policy", "ignore");
    params_.traversability_unknown_cost = static_cast<int>(declare_parameter<int>(
      "traversability_unknown_cost", 30));
    params_.traversability_max_slope_deg = declare_parameter<double>(
      "traversability_max_slope_deg", 18.0);
    params_.traversability_max_roughness_m = declare_parameter<double>(
      "traversability_max_roughness_m", 0.06);
    params_.traversability_max_step_height_m = declare_parameter<double>(
      "traversability_max_step_height_m", 0.10);
    params_.traversability_obstacle_density_threshold = declare_parameter<double>(
      "traversability_obstacle_density_threshold", 0.25);
    params_.traversability_debug_enabled = declare_parameter<bool>(
      "traversability_debug_enabled", false);
    params_.traversability_debug_publish_hz = declare_parameter<double>(
      "traversability_debug_publish_hz", 1.0);

    debug_enabled_ = params_.traversability_debug_enabled;
    debug_publish_hz_ = std::max(0.1, params_.traversability_debug_publish_hz);

    target_frame_ = declare_parameter<std::string>("target_frame", "map");
    const auto legacy_frame_id = declare_parameter<std::string>("frame_id", "");
    if (!legacy_frame_id.empty()) {
      target_frame_ = legacy_frame_id;
    }
    classification_frame_ = declare_parameter<std::string>("classification_frame", "base_link");
    if (classification_frame_.empty()) {
      classification_frame_ = target_frame_;
    }
    classification_z_offset_m_ = declare_parameter<double>("classification_z_offset_m", 0.0);
    classification_ground_plane_enabled_ = declare_parameter<bool>(
      "classification_ground_plane_enabled", false);
    classification_ground_plane_a_ = declare_parameter<double>("classification_ground_plane_a", 0.0);
    classification_ground_plane_b_ = declare_parameter<double>("classification_ground_plane_b", 0.0);
    classification_ground_plane_c_ = declare_parameter<double>("classification_ground_plane_c", 0.0);
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.2);

    input_min_range_m_ = std::max(0.0, declare_parameter<double>("input_min_range_m", 0.15));
    self_filter_enabled_ = declare_parameter<bool>("self_filter_enabled", true);
    self_filter_frame_ = declare_parameter<std::string>("self_filter_frame", "base_link");
    self_filter_min_x_ = declare_parameter<double>("self_filter_min_x", -0.45);
    self_filter_max_x_ = declare_parameter<double>("self_filter_max_x", 0.45);
    self_filter_min_y_ = declare_parameter<double>("self_filter_min_y", -0.35);
    self_filter_max_y_ = declare_parameter<double>("self_filter_max_y", 0.35);
    self_filter_min_z_ = declare_parameter<double>("self_filter_min_z", -0.20);
    self_filter_max_z_ = declare_parameter<double>("self_filter_max_z", 0.45);

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

  Eigen::Isometry3f lookup_isometry(
    const std::string & target_frame,
    const std::string & source_frame,
    const rclcpp::Time & stamp)
  {
    if (target_frame == source_frame || target_frame.empty()) {
      return Eigen::Isometry3f::Identity();
    }

    const auto transform = tf_buffer_->lookupTransform(
      target_frame, source_frame, stamp,
      rclcpp::Duration::from_seconds(transform_timeout_sec_));
    return transform_to_isometry(transform);
  }

  bool inside_self_filter_box(const Eigen::Vector3f & p) const
  {
    return p.x() >= self_filter_min_x_ && p.x() <= self_filter_max_x_ &&
           p.y() >= self_filter_min_y_ && p.y() <= self_filter_max_y_ &&
           p.z() >= self_filter_min_z_ && p.z() <= self_filter_max_z_;
  }

  FilteredCloud filter_and_transform(
    const Eigen::Ref<const Eigen::MatrixX3f> & raw_xyz,
    const Eigen::Isometry3f & classification_from_source,
    const Eigen::Isometry3f & target_from_classification,
    const Eigen::Isometry3f & self_from_source,
    bool use_self_filter,
    std::size_t & dropped_min_range,
    std::size_t & dropped_self_filter) const
  {
    dropped_min_range = 0;
    dropped_self_filter = 0;
    Eigen::MatrixX3f classification_out(raw_xyz.rows(), 3);
    Eigen::MatrixX3f target_out(raw_xyz.rows(), 3);
    Eigen::Index w = 0;

    for (Eigen::Index i = 0; i < raw_xyz.rows(); ++i) {
      const Eigen::Vector3f raw = raw_xyz.row(i).transpose();
      if (!raw.allFinite()) {
        continue;
      }
      if (input_min_range_m_ > 0.0 && raw.norm() < static_cast<float>(input_min_range_m_)) {
        ++dropped_min_range;
        continue;
      }

      if (use_self_filter) {
        const Eigen::Vector3f self = self_from_source * raw;
        if (inside_self_filter_box(self)) {
          ++dropped_self_filter;
          continue;
        }
      }

      const Eigen::Vector3f physical_classification = classification_from_source * raw;
      Eigen::Vector3f classification = physical_classification;
      if (classification_ground_plane_enabled_) {
        const float ground_z =
          static_cast<float>(
          classification_ground_plane_a_ * static_cast<double>(classification.x()) +
          classification_ground_plane_b_ * static_cast<double>(classification.y()) +
          classification_ground_plane_c_);
        classification.z() -= ground_z;
      }
      classification.z() += static_cast<float>(classification_z_offset_m_);
      const Eigen::Vector3f target = target_from_classification * physical_classification;
      classification_out.row(w) = classification.transpose();
      target_out.row(w) = target.transpose();
      ++w;
    }

    classification_out.conservativeResize(w, 3);
    target_out.conservativeResize(w, 3);
    return FilteredCloud{classification_out, target_out};
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
      const auto source_frame = msg->header.frame_id;
      if (source_frame.empty()) {
        tf_ok_ = false;
        tf_reason_ = "missing_source_frame";
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Cannot transform pointcloud with empty frame_id to target frame '%s'",
          target_frame_.c_str());
        return;
      }

      const auto effective_target_frame = target_frame_.empty() ? source_frame : target_frame_;
      const auto effective_classification_frame =
        classification_frame_.empty() ? effective_target_frame : classification_frame_;
      Eigen::Isometry3f classification_from_source = Eigen::Isometry3f::Identity();
      Eigen::Isometry3f target_from_classification = Eigen::Isometry3f::Identity();
      Eigen::Isometry3f self_from_source = Eigen::Isometry3f::Identity();
      try {
        classification_from_source = lookup_isometry(
          effective_classification_frame, source_frame, msg->header.stamp);
        target_from_classification = lookup_isometry(
          effective_target_frame, effective_classification_frame, msg->header.stamp);
        if (self_filter_enabled_ && !self_filter_frame_.empty()) {
          self_from_source = lookup_isometry(self_filter_frame_, source_frame, msg->header.stamp);
        }
      } catch (const tf2::TransformException & ex) {
        tf_ok_ = false;
        tf_reason_ = "tf_unavailable";
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "TF unavailable for ground segmentation: %s -> target=%s self=%s: %s",
          source_frame.c_str(), effective_target_frame.c_str(), self_filter_frame_.c_str(),
          ex.what());
        return;
      }

      Eigen::MatrixX3f raw_xyz = extract_xyz(*msg);
      last_input_points_ = static_cast<std::size_t>(raw_xyz.rows());
      std::size_t dropped_min_range = 0;
      std::size_t dropped_self_filter = 0;
      FilteredCloud filtered = filter_and_transform(
        raw_xyz, classification_from_source, target_from_classification, self_from_source,
        self_filter_enabled_ && !self_filter_frame_.empty(),
        dropped_min_range, dropped_self_filter);
      Eigen::MatrixX3f & classification_xyz = filtered.classification_xyz;
      Eigen::MatrixX3f & target_xyz = filtered.target_xyz;
      last_filtered_points_ = static_cast<std::size_t>(classification_xyz.rows());
      last_dropped_min_range_ = dropped_min_range;
      last_dropped_self_filter_ = dropped_self_filter;

      tf_ok_ = true;
      tf_reason_ = "ok";

      std_msgs::msg::Header out_header = msg->header;
      out_header.frame_id = effective_target_frame;
      const auto N = static_cast<std::size_t>(classification_xyz.rows());
      if (N < 10) {
        Eigen::MatrixX3f empty(0, 3);
        ground_pub_->publish(build_xyz_cloud(out_header, empty));
        obstacle_pub_->publish(build_xyz_cloud(out_header, empty));
        ++empty_frames_;
        consecutive_failures_ = 0;
        return;
      }
      auto cls = segmenter_->classify(classification_xyz);

      // Split ground / obstacle, preserving original order.
      Eigen::MatrixX3f ground(static_cast<Eigen::Index>(cls.ground_count), 3);
      Eigen::MatrixX3f obstacle(
        static_cast<Eigen::Index>(N - cls.ground_count), 3);
      Eigen::Index gi = 0;
      Eigen::Index oi = 0;
      for (std::size_t i = 0; i < N; ++i) {
        const auto row = static_cast<Eigen::Index>(i);
        if (cls.ground_mask[i]) {
          ground.row(gi++) = target_xyz.row(row);
        } else {
          obstacle.row(oi++) = target_xyz.row(row);
        }
      }

      ground_pub_->publish(build_xyz_cloud(out_header, ground));
      obstacle_pub_->publish(build_xyz_cloud(out_header, obstacle));

      // Accumulate traversability.
      if (params_.traversability_v2_enabled) {
        const double now_sec =
          static_cast<double>(std::chrono::steady_clock::now().time_since_epoch().count()) * 1e-9;
        segmenter_->accumulate_traversability_v2(ground, obstacle, now_sec);
      } else {
        segmenter_->accumulate_traversability(ground);
      }

      const auto t1 = std::chrono::steady_clock::now();
      const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
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

  void publish_v2_grid(const std::string & layer, const std::string & suffix)
  {
    const double now_sec =
      static_cast<double>(std::chrono::steady_clock::now().time_since_epoch().count()) * 1e-9;
    std::vector<int8_t> data;
    if (!segmenter_->render_cost_layer(data, layer, now_sec)) {
      return;
    }
    auto grid = make_grid_header(
      target_frame_,
      params_.traversability_resolution,
      params_.traversability_width,
      params_.traversability_height,
      params_.traversability_origin_x,
      params_.traversability_origin_y,
      now());
    grid.data = std::move(data);
    if (suffix == "/slope" && debug_slope_pub_) {
      debug_slope_pub_->publish(grid);
    } else if (suffix == "/roughness" && debug_roughness_pub_) {
      debug_roughness_pub_->publish(grid);
    } else if (suffix == "/step" && debug_step_pub_) {
      debug_step_pub_->publish(grid);
    } else if (suffix == "/obstacle_density" && debug_obstacle_density_pub_) {
      debug_obstacle_density_pub_->publish(grid);
    } else if (suffix == "/confidence" && debug_confidence_pub_) {
      debug_confidence_pub_->publish(grid);
    } else if (suffix == "/reason" && debug_reason_pub_) {
      debug_reason_pub_->publish(grid);
    }
  }

  void on_traversability_timer()
  {
    try {
      if (!tf_ok_) {
        return;
      }

      const auto t0 = std::chrono::steady_clock::now();
      const double now_sec =
        static_cast<double>(std::chrono::steady_clock::now().time_since_epoch().count()) * 1e-9;

      if (params_.traversability_v2_enabled) {
        // V2 main grid: graded final cost.
        std::vector<int8_t> data;
        if (segmenter_->render_traversability_v2(data, now_sec)) {
          auto grid = make_grid_header(
            target_frame_,
            params_.traversability_resolution,
            params_.traversability_width,
            params_.traversability_height,
            params_.traversability_origin_x,
            params_.traversability_origin_y,
            now());
          grid.data = std::move(data);
          traversability_pub_->publish(grid);
        }
        const auto t1 = std::chrono::steady_clock::now();
        last_render_ms_ = std::chrono::duration<double, std::milli>(t1 - t0).count();

        // V2 debug layers — only when enabled and at the configured rate.
        double debug_ms = 0.0;
        if (debug_enabled_) {
          const auto now = this->now();
          const double dt = (now - last_debug_publish_time_).seconds();
          if (dt >= (1.0 / debug_publish_hz_)) {
            const auto dt0 = std::chrono::steady_clock::now();
            publish_v2_grid("slope", "/slope");
            publish_v2_grid("roughness", "/roughness");
            publish_v2_grid("step", "/step");
            publish_v2_grid("obstacle_density", "/obstacle_density");
            publish_v2_grid("confidence", "/confidence");
            publish_v2_grid("reason", "/reason");
            const auto dt1 = std::chrono::steady_clock::now();
            debug_ms = std::chrono::duration<double, std::milli>(dt1 - dt0).count();
            last_debug_publish_time_ = now;
          }
        }
        last_debug_ms_ = debug_ms;

        // Cache V2 stats for status messages. (Re-use costs computed during render.)
        const auto stats = segmenter_->get_v2_stats(now_sec);
        last_v2_known_cells_ = stats.known_cells;
        last_v2_unknown_cells_ = stats.unknown_cells;
        last_v2_stale_cells_ = stats.stale_cells;
        last_v2_high_cost_cells_ = stats.high_cost_cells;
        last_v2_max_cost_ = stats.max_cost;
        last_v2_mean_confidence_ = stats.mean_confidence;
      } else {
        std::vector<int8_t> data;
        if (!segmenter_->render_traversability(data)) {
          return;
        }
        auto grid = make_grid_header(
          target_frame_,
          params_.traversability_resolution,
          params_.traversability_width,
          params_.traversability_height,
          params_.traversability_origin_x,
          params_.traversability_origin_y,
          now());
        grid.data = std::move(data);
        traversability_pub_->publish(grid);
        const auto t1 = std::chrono::steady_clock::now();
        last_render_ms_ = std::chrono::duration<double, std::milli>(t1 - t0).count();
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ground_segmentation_cpp traversability publish error: %s", e.what());
    }
  }

  void publish_status()
  {
    std_msgs::msg::String msg;
    const bool error = (consecutive_failures_ >= max_consecutive_failures_) || !tf_ok_;
    const char * state = error ? (tf_ok_ ? "error" : "tf_error") :
      (processed_frames_ > 0 ? "ready" : "waiting_cloud");
    const char * reason = !tf_ok_ ? tf_reason_.c_str() :
      (consecutive_failures_ >= max_consecutive_failures_ ? "consecutive_segmentation_failures" :
        (processed_frames_ > 0 ? "ok" : "no_cloud_yet"));

    const char * unknown_policy_str = params_.traversability_unknown_policy.c_str();

    char buf[1024];
    if (params_.traversability_v2_enabled) {
      std::snprintf(
        buf, sizeof(buf),
        "mode=cpp_v2;state=%s;ready=%s;reason=%s;processed=%lu;skipped=%lu;empty=%lu;"
        "exceptions=%lu;latency_ms_ewma=%.2f;known_cells=%lu;input_points=%lu;"
        "filtered_points=%lu;dropped_min_range=%lu;dropped_self_filter=%lu;"
        "traversability_v2=true;"
        "known_cells_v2=%lu;unknown_cells_v2=%lu;stale_cells=%lu;"
        "high_cost_cells=%lu;max_cost=%.1f;mean_confidence=%.3f;"
        "unknown_policy=%s;debug_enabled=%s;"
        "render_ms=%.1f;debug_publish_ms=%.1f",
        state,
        (error || processed_frames_ == 0) ? "false" : "true",
        reason,
        static_cast<unsigned long>(processed_frames_),
        static_cast<unsigned long>(skipped_frames_),
        static_cast<unsigned long>(empty_frames_),
        static_cast<unsigned long>(exception_frames_),
        latency_ms_ema_,
        static_cast<unsigned long>(segmenter_ ? segmenter_->known_cell_count() : 0),
        static_cast<unsigned long>(last_input_points_),
        static_cast<unsigned long>(last_filtered_points_),
        static_cast<unsigned long>(last_dropped_min_range_),
        static_cast<unsigned long>(last_dropped_self_filter_),
        static_cast<unsigned long>(last_v2_known_cells_),
        static_cast<unsigned long>(last_v2_unknown_cells_),
        static_cast<unsigned long>(last_v2_stale_cells_),
        static_cast<unsigned long>(last_v2_high_cost_cells_),
        static_cast<double>(last_v2_max_cost_),
        static_cast<double>(last_v2_mean_confidence_),
        unknown_policy_str,
        debug_enabled_ ? "true" : "false",
        last_render_ms_,
        last_debug_ms_);
    } else {
      std::snprintf(
        buf, sizeof(buf),
        "mode=cpp;state=%s;ready=%s;reason=%s;processed=%lu;skipped=%lu;empty=%lu;"
        "exceptions=%lu;latency_ms_ewma=%.2f;known_cells=%lu;input_points=%lu;"
        "filtered_points=%lu;dropped_min_range=%lu;dropped_self_filter=%lu",
        state,
        (error || processed_frames_ == 0) ? "false" : "true",
        reason,
        static_cast<unsigned long>(processed_frames_),
        static_cast<unsigned long>(skipped_frames_),
        static_cast<unsigned long>(empty_frames_),
        static_cast<unsigned long>(exception_frames_),
        latency_ms_ema_,
        static_cast<unsigned long>(segmenter_ ? segmenter_->known_cell_count() : 0),
        static_cast<unsigned long>(last_input_points_),
        static_cast<unsigned long>(last_filtered_points_),
        static_cast<unsigned long>(last_dropped_min_range_),
        static_cast<unsigned long>(last_dropped_self_filter_));
    }
    msg.data = buf;
    status_pub_->publish(msg);
  }

  // ── Params ───────────────────────────────────────────────────────────
  a2gs::GroundSegmenterParams params_;
  std::string input_topic_;
  std::string ground_topic_;
  std::string obstacle_topic_;
  std::string traversability_topic_;
  std::string status_topic_;
  std::string debug_topic_prefix_;
  std::string target_frame_;
  std::string classification_frame_;
  double classification_z_offset_m_{0.0};
  bool classification_ground_plane_enabled_{false};
  double classification_ground_plane_a_{0.0};
  double classification_ground_plane_b_{0.0};
  double classification_ground_plane_c_{0.0};
  double transform_timeout_sec_{0.2};
  double input_min_range_m_{0.15};
  bool self_filter_enabled_{true};
  std::string self_filter_frame_{"base_link"};
  double self_filter_min_x_{-0.45};
  double self_filter_max_x_{0.45};
  double self_filter_min_y_{-0.35};
  double self_filter_max_y_{0.35};
  double self_filter_min_z_{-0.20};
  double self_filter_max_z_{0.45};
  int process_every_n_{1};
  int max_consecutive_failures_{5};

  // ── State ─────────────────────────────────────────────────────────────
  std::unique_ptr<a2gs::GroundSegmenter> segmenter_;
  std::uint64_t recv_count_{0};
  std::uint64_t processed_frames_{0};
  std::uint64_t skipped_frames_{0};
  std::uint64_t empty_frames_{0};
  std::uint64_t exception_frames_{0};
  int consecutive_failures_{0};
  double latency_ms_ema_{-1.0};
  bool tf_ok_{true};
  std::string tf_reason_{"ok"};
  std::size_t last_input_points_{0};
  std::size_t last_filtered_points_{0};
  std::size_t last_dropped_min_range_{0};
  std::size_t last_dropped_self_filter_{0};

  // V2 cached stats (populated during traversability publish).
  std::size_t last_v2_known_cells_{0};
  std::size_t last_v2_unknown_cells_{0};
  std::size_t last_v2_stale_cells_{0};
  std::size_t last_v2_high_cost_cells_{0};
  float last_v2_max_cost_{0.0f};
  float last_v2_mean_confidence_{0.0f};

  // Timings.
  double last_render_ms_{0.0};
  double last_debug_ms_{0.0};

  // Debug state.
  bool debug_enabled_{false};
  double debug_publish_hz_{1.0};
  rclcpp::Time last_debug_publish_time_{0, 0, RCL_ROS_TIME};

  // ── ROS ───────────────────────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr traversability_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  // V2 debug publishers.
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_slope_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_roughness_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_step_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_obstacle_density_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_confidence_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_reason_pub_;

  rclcpp::TimerBase::SharedPtr traversability_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
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
