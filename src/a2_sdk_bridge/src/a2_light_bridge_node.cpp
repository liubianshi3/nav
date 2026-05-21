#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "a2_interfaces/msg/light_command.hpp"
#include "a2_unitree_ipc/client.hpp"
#include "a2_unitree_ipc/protocol.hpp"
#include "rclcpp/rclcpp.hpp"

class A2LightBridgeNode : public rclcpp::Node
{
public:
  A2LightBridgeNode()
  : Node("a2_light_bridge")
  {
    use_mock_ = declare_parameter<bool>("use_mock", true);
    command_topic_ = declare_parameter<std::string>("command_topic", "/a2/light/command");
    ipc_socket_path_ = declare_parameter<std::string>("ipc_socket_path", a2_unitree_ipc::kDefaultSocketPath);
    ipc_timeout_ms_ = declare_parameter<int>("ipc_timeout_ms", 200);
    declare_parameter<std::string>("runtime_mode", use_mock_ ? "mock" : "real");
    declare_parameter<bool>("auto_detect_interface", true);
    declare_parameter<bool>("allow_loopback", false);
    declare_parameter<std::string>("network_interface", "");
    declare_parameter<std::vector<std::string>>("interface_candidates", std::vector<std::string>{});
    declare_parameter<std::string>("lowcmd_topic", "rt/lowcmd");
    declare_parameter<int>("send_repeat", 3);
    declare_parameter<double>("send_hz", 10.0);
    declare_parameter<bool>("use_sim_time", false);

    sub_ = create_subscription<a2_interfaces::msg::LightCommand>(
      command_topic_, 10,
      std::bind(&A2LightBridgeNode::on_command, this, std::placeholders::_1));
  }

private:
  a2_unitree_ipc::UnixSocketClient & ipc_client()
  {
    if (!ipc_client_) {
      ipc_client_ = std::make_unique<a2_unitree_ipc::UnixSocketClient>(ipc_socket_path_, ipc_timeout_ms_);
    }
    return *ipc_client_;
  }

  uint64_t next_seq()
  {
    return ++ipc_seq_;
  }

  void on_command(const a2_interfaces::msg::LightCommand::SharedPtr msg)
  {
    if (use_mock_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000, "Mock light command accepted.");
      return;
    }

    a2_unitree_ipc::LightCommand command;
    command.seq = next_seq();
    command.on = msg->on;
    command.color_mode = msg->color_mode;
    command.intensity = std::max(0, std::min(255, static_cast<int>(msg->intensity)));
    command.r = std::max(0, std::min(255, static_cast<int>(msg->r)));
    command.g = std::max(0, std::min(255, static_cast<int>(msg->g)));
    command.b = std::max(0, std::min(255, static_cast<int>(msg->b)));
    command.color_temperature_kelvin = msg->color_temperature_kelvin;

    std::lock_guard<std::mutex> guard(ipc_mutex_);
    std::string error;
    auto & client = ipc_client();
    if (!client.send_message(a2_unitree_ipc::encode_light_command(command), &error)) {
      client.close();
      RCLCPP_WARN(get_logger(), "Failed to send light command to unitree_agent: %s", error.c_str());
      return;
    }

    std::string response;
    if (!client.read_message(&response, ipc_timeout_ms_, &error)) {
      client.close();
      RCLCPP_WARN(get_logger(), "Failed to read light ACK from unitree_agent: %s", error.c_str());
      return;
    }
    a2_unitree_ipc::Ack ack;
    if (!a2_unitree_ipc::decode_ack(response, &ack) || !ack.ok) {
      RCLCPP_WARN(get_logger(), "unitree_agent rejected light command: %s", response.c_str());
    }
  }

  bool use_mock_{true};
  std::string command_topic_;
  std::string ipc_socket_path_{a2_unitree_ipc::kDefaultSocketPath};
  int ipc_timeout_ms_{200};
  uint64_t ipc_seq_{0};

  rclcpp::Subscription<a2_interfaces::msg::LightCommand>::SharedPtr sub_;
  std::unique_ptr<a2_unitree_ipc::UnixSocketClient> ipc_client_;
  std::mutex ipc_mutex_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<A2LightBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
