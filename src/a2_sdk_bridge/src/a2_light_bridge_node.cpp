#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "a2_interfaces/msg/light_command.hpp"
#include "a2_system/network_utils.hpp"
#include "rclcpp/rclcpp.hpp"

#if A2_ENABLE_UNITREE_SDK
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#endif

class A2LightBridgeNode : public rclcpp::Node
{
public:
  A2LightBridgeNode()
  : Node("a2_light_bridge")
  {
    use_mock_ = declare_parameter<bool>("use_mock", true);
    runtime_mode_ = declare_parameter<std::string>("runtime_mode", use_mock_ ? "mock" : "real");
    auto_detect_interface_ = declare_parameter<bool>("auto_detect_interface", true);
    allow_loopback_ = declare_parameter<bool>("allow_loopback", true);
    network_interface_ = declare_parameter<std::string>("network_interface", "");
    interface_candidates_ = declare_parameter<std::vector<std::string>>(
      "interface_candidates", std::vector<std::string>{});
    command_topic_ = declare_parameter<std::string>("command_topic", "/a2/light/command");
    lowcmd_topic_ = declare_parameter<std::string>("lowcmd_topic", "rt/lowcmd");
    send_repeat_ = declare_parameter<int>("send_repeat", 5);
    send_hz_ = declare_parameter<double>("send_hz", 10.0);
    declare_parameter<bool>("use_sim_time", false);

    resolved_interface_ = resolve_interface();

#if A2_ENABLE_UNITREE_SDK
    if (!use_mock_ && runtime_mode_ == "real") {
      if (resolved_interface_.empty()) {
        sdk_ready_ = false;
        RCLCPP_ERROR(get_logger(), "No usable network interface available for real light control.");
      } else if (!a2_system::interface_is_ready_for_real(resolved_interface_)) {
        sdk_ready_ = false;
        RCLCPP_WARN(
          get_logger(),
          "Interface '%s' is not ready for real light control. Light bridge will stay disabled.",
          resolved_interface_.c_str());
      } else {
        unitree::robot::ChannelFactory::Instance()->Init(0, resolved_interface_);
        lowcmd_pub_ = std::make_unique<unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowCmd_>>(
          lowcmd_topic_);
        lowcmd_pub_->InitChannel();
        sdk_ready_ = true;
        RCLCPP_INFO(
          get_logger(),
          "A2 light bridge initialized on interface '%s', topic '%s'.",
          resolved_interface_.c_str(),
          lowcmd_topic_.c_str());
      }
    }
#endif

    sub_ = create_subscription<a2_interfaces::msg::LightCommand>(
      command_topic_, 10,
      std::bind(&A2LightBridgeNode::on_command, this, std::placeholders::_1));

    const double hz = std::max(1.0, send_hz_);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / hz)),
      std::bind(&A2LightBridgeNode::tick, this));
  }

private:
  std::string resolve_interface() const
  {
    if (runtime_mode_ == "gazebo") {
      return "gazebo";
    }
    const bool simulated_mode = runtime_mode_ == "mock" || runtime_mode_ == "gazebo";
    const bool allow_loopback = simulated_mode && allow_loopback_;
    if (!network_interface_.empty() && a2_system::interface_exists(network_interface_)) {
      return network_interface_;
    }
    if (auto_detect_interface_) {
      return a2_system::select_interface(network_interface_, interface_candidates_, allow_loopback);
    }
    return network_interface_;
  }

  static uint8_t clamp_u8(int value)
  {
    return static_cast<uint8_t>(std::max(0, std::min(255, value)));
  }

  static std::array<uint8_t, 3> kelvin_to_rgb(uint16_t kelvin)
  {
    double temp = std::max(1000.0, std::min(40000.0, static_cast<double>(kelvin))) / 100.0;
    double r;
    double g;
    double b;
    if (temp <= 66.0) {
      r = 255.0;
      g = 99.4708025861 * std::log(temp) - 161.1195681661;
      if (temp <= 19.0) {
        b = 0.0;
      } else {
        b = 138.5177312231 * std::log(temp - 10.0) - 305.0447927307;
      }
    } else {
      r = 329.698727446 * std::pow(temp - 60.0, -0.1332047592);
      g = 288.1221695283 * std::pow(temp - 60.0, -0.0755148492);
      b = 255.0;
    }
    return {
      clamp_u8(static_cast<int>(std::lround(std::max(0.0, std::min(255.0, r))))),
      clamp_u8(static_cast<int>(std::lround(std::max(0.0, std::min(255.0, g))))),
      clamp_u8(static_cast<int>(std::lround(std::max(0.0, std::min(255.0, b))))),
    };
  }

  void on_command(const a2_interfaces::msg::LightCommand::SharedPtr msg)
  {
    const bool on = msg->on;
    const int intensity = std::max(0, std::min(255, static_cast<int>(msg->intensity)));
    std::array<uint8_t, 3> rgb = {0U, 0U, 0U};

    if (on) {
      if (msg->color_mode == 1) {
        rgb = {static_cast<uint8_t>(intensity), static_cast<uint8_t>(intensity), static_cast<uint8_t>(intensity)};
      } else if (msg->color_mode == 2) {
        const double scale = static_cast<double>(intensity) / 255.0;
        rgb = {
          clamp_u8(static_cast<int>(std::lround(static_cast<double>(msg->r) * scale))),
          clamp_u8(static_cast<int>(std::lround(static_cast<double>(msg->g) * scale))),
          clamp_u8(static_cast<int>(std::lround(static_cast<double>(msg->b) * scale))),
        };
      } else if (msg->color_mode == 3) {
        const auto base = kelvin_to_rgb(msg->color_temperature_kelvin);
        const double scale = static_cast<double>(intensity) / 255.0;
        rgb = {
          clamp_u8(static_cast<int>(std::lround(static_cast<double>(base[0]) * scale))),
          clamp_u8(static_cast<int>(std::lround(static_cast<double>(base[1]) * scale))),
          clamp_u8(static_cast<int>(std::lround(static_cast<double>(base[2]) * scale))),
        };
      } else {
        rgb = {static_cast<uint8_t>(intensity), static_cast<uint8_t>(intensity), static_cast<uint8_t>(intensity)};
      }
    }

    {
      std::lock_guard<std::mutex> guard(mutex_);
      current_rgb_ = rgb;
      pending_sends_ = std::max(1, send_repeat_);
    }
  }

#if A2_ENABLE_UNITREE_SDK
  static void append_u8(std::vector<uint8_t> & out, uint8_t value)
  {
    out.push_back(value);
  }

  static void append_u16_le(std::vector<uint8_t> & out, uint16_t value)
  {
    out.push_back(static_cast<uint8_t>(value & 0xFF));
    out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
  }

  static void append_u32_le(std::vector<uint8_t> & out, uint32_t value)
  {
    out.push_back(static_cast<uint8_t>(value & 0xFF));
    out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
    out.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
    out.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
  }

  static void append_f32_le(std::vector<uint8_t> & out, float value)
  {
    static_assert(sizeof(float) == 4);
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    append_u32_le(out, bits);
  }

  static uint32_t crc32_words(const std::vector<uint32_t> & words)
  {
    uint32_t crc = 0xFFFFFFFFu;
    constexpr uint32_t polynomial = 0x04c11db7u;
    for (uint32_t current : words) {
      uint32_t bit = 1u << 31;
      for (int i = 0; i < 32; ++i) {
        if (crc & 0x80000000u) {
          crc = (crc << 1) & 0xFFFFFFFFu;
          crc ^= polynomial;
        } else {
          crc = (crc << 1) & 0xFFFFFFFFu;
        }
        if (current & bit) {
          crc ^= polynomial;
        }
        bit >>= 1;
      }
    }
    return crc;
  }

  static uint32_t compute_lowcmd_crc(const unitree_go::msg::dds_::LowCmd_ & cmd)
  {
    std::vector<uint8_t> packed;
    packed.reserve(812);
    append_u8(packed, cmd.head()[0]);
    append_u8(packed, cmd.head()[1]);
    append_u8(packed, cmd.level_flag());
    append_u8(packed, cmd.frame_reserve());
    append_u32_le(packed, cmd.sn()[0]);
    append_u32_le(packed, cmd.sn()[1]);
    append_u32_le(packed, cmd.version()[0]);
    append_u32_le(packed, cmd.version()[1]);
    append_u16_le(packed, cmd.bandwidth());
    append_u8(packed, 0);
    append_u8(packed, 0);
    for (std::size_t i = 0; i < cmd.motor_cmd().size(); ++i) {
      const auto & motor = cmd.motor_cmd()[i];
      append_u8(packed, motor.mode());
      append_u8(packed, 0);
      append_u8(packed, 0);
      append_u8(packed, 0);
      append_f32_le(packed, motor.q());
      append_f32_le(packed, motor.dq());
      append_f32_le(packed, motor.tau());
      append_f32_le(packed, motor.kp());
      append_f32_le(packed, motor.kd());
      append_u32_le(packed, motor.reserve()[0]);
      append_u32_le(packed, motor.reserve()[1]);
      append_u32_le(packed, motor.reserve()[2]);
    }
    append_u8(packed, cmd.bms_cmd().off());
    append_u8(packed, cmd.bms_cmd().reserve()[0]);
    append_u8(packed, cmd.bms_cmd().reserve()[1]);
    append_u8(packed, cmd.bms_cmd().reserve()[2]);
    for (auto v : cmd.wireless_remote()) {
      append_u8(packed, v);
    }
    for (auto v : cmd.led()) {
      append_u8(packed, v);
    }
    append_u8(packed, cmd.fan()[0]);
    append_u8(packed, cmd.fan()[1]);
    append_u8(packed, cmd.gpio());
    append_u8(packed, 0);
    append_u8(packed, 0);
    append_u32_le(packed, cmd.reserve());
    append_u32_le(packed, 0);

    const std::size_t word_count = (packed.size() / 4);
    if (word_count < 2) {
      return 0;
    }
    std::vector<uint32_t> words;
    words.reserve(word_count - 1);
    for (std::size_t idx = 0; idx < word_count - 1; ++idx) {
      const std::size_t base = idx * 4;
      const uint32_t d =
        (static_cast<uint32_t>(packed[base + 3]) << 24) |
        (static_cast<uint32_t>(packed[base + 2]) << 16) |
        (static_cast<uint32_t>(packed[base + 1]) << 8) |
        static_cast<uint32_t>(packed[base]);
      words.push_back(d);
    }
    return crc32_words(words);
  }
#endif

  void tick()
  {
    std::array<uint8_t, 3> rgb;
    int remaining = 0;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      rgb = current_rgb_;
      remaining = pending_sends_;
      if (pending_sends_ > 0) {
        pending_sends_ -= 1;
      }
    }
    if (remaining <= 0) {
      return;
    }

#if A2_ENABLE_UNITREE_SDK
    if (!sdk_ready_ || !lowcmd_pub_) {
      return;
    }
    unitree_go::msg::dds_::LowCmd_ cmd;
    cmd.head()[0] = 0xFE;
    cmd.head()[1] = 0xEF;
    cmd.level_flag() = 0xFF;
    cmd.frame_reserve() = 0;
    cmd.gpio() = 0;
    for (int i = 0; i < 4; ++i) {
      cmd.led()[i * 3 + 0] = rgb[0];
      cmd.led()[i * 3 + 1] = rgb[1];
      cmd.led()[i * 3 + 2] = rgb[2];
    }
    cmd.crc(compute_lowcmd_crc(cmd));
    lowcmd_pub_->Write(cmd);
#else
    (void)rgb;
#endif
  }

  bool use_mock_{true};
  bool auto_detect_interface_{true};
  bool allow_loopback_{true};
  bool sdk_ready_{false};
  std::string runtime_mode_;
  std::string network_interface_;
  std::vector<std::string> interface_candidates_;
  std::string resolved_interface_;
  std::string command_topic_;
  std::string lowcmd_topic_;
  int send_repeat_{5};
  double send_hz_{10.0};

  rclcpp::Subscription<a2_interfaces::msg::LightCommand>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  std::array<uint8_t, 3> current_rgb_{0U, 0U, 0U};
  int pending_sends_{0};

#if A2_ENABLE_UNITREE_SDK
  std::unique_ptr<unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowCmd_>> lowcmd_pub_;
#endif
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<A2LightBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
