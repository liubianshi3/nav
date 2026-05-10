#ifndef A2_NDT_SCAN_MATCHER__DIAGNOSTICS_INTERFACE_MOCK_HPP_
#define A2_NDT_SCAN_MATCHER__DIAGNOSTICS_INTERFACE_MOCK_HPP_

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <string>
#include <vector>

namespace autoware_utils_diagnostics
{
class DiagnosticsInterface
{
public:
  DiagnosticsInterface(rclcpp::Node * node, const std::string & name) : node_(node), name_(name) {
    status_.name = name_;
    status_.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status_.message = "OK";
  }

  void clear() {
    status_.values.clear();
    status_.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status_.message = "OK";
  }

  template <typename T>
  void add_key_value(const std::string & key, const T & value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    if constexpr (std::is_same_v<T, std::string> || std::is_same_v<T, const char *>) {
      kv.value = value;
    } else {
      kv.value = std::to_string(value);
    }
    status_.values.push_back(kv);
  }

  void update_level_and_message(int level, const std::string & message) {
    status_.level = level;
    status_.message = message;
  }

  void publish(const rclcpp::Time & /*time*/) {
    // In a real node, this would use diagnostic_updater. 
    // Here we just keep the API satisfied.
  }

private:
  rclcpp::Node * node_;
  std::string name_;
  diagnostic_msgs::msg::DiagnosticStatus status_;
};
}  // namespace autoware_utils_diagnostics

#endif  // A2_NDT_SCAN_MATCHER__DIAGNOSTICS_INTERFACE_MOCK_HPP_
