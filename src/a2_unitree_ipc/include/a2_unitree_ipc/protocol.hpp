#ifndef A2_UNITREE_IPC_PROTOCOL_HPP_
#define A2_UNITREE_IPC_PROTOCOL_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>

namespace a2_unitree_ipc
{

constexpr const char * kDefaultSocketPath = "/run/a2/unitree_agent.sock";

using Fields = std::unordered_map<std::string, std::string>;

struct ControlCommand
{
  std::uint64_t seq{0};
  double linear_x{0.0};
  double linear_y{0.0};
  double angular_z{0.0};
  int timeout_ms{300};
  int gait_type{1};
  int speed_level{1};
  double body_height{0.0};
  bool auto_recovery{false};
};

struct StopCommand
{
  std::uint64_t seq{0};
  std::string reason{"unspecified"};
};

struct MotionCommand
{
  std::uint64_t seq{0};
  std::string command;
  int int_value{0};
  double float_value{0.0};
  bool bool_value{false};
};

struct LightCommand
{
  std::uint64_t seq{0};
  bool on{false};
  int color_mode{0};
  int intensity{0};
  int r{0};
  int g{0};
  int b{0};
  int color_temperature_kelvin{4500};
};

struct Ack
{
  std::uint64_t seq{0};
  bool ok{false};
  int code{0};
  std::string message;
};

struct HealthStatus
{
  bool connected{false};
  bool sdk_ready{false};
  bool ipc_ready{false};
  std::string state{"unknown"};
  std::string reason{"unknown"};
  std::string last_stop_reason{"none"};
};

struct StateStream
{
  std::uint64_t seq{0};
  std::string source_mode{"unknown"};
  bool connected{false};
  bool imu_valid{false};
  bool odom_valid{false};
  std::array<float, 3> position{0.0F, 0.0F, 0.0F};
  std::array<float, 3> velocity{0.0F, 0.0F, 0.0F};
  std::array<float, 4> orientation_xyzw{0.0F, 0.0F, 0.0F, 1.0F};
  std::array<float, 3> rpy{0.0F, 0.0F, 0.0F};
  std::array<float, 3> linear_acceleration{0.0F, 0.0F, 0.0F};
  std::array<float, 3> angular_velocity{0.0F, 0.0F, 0.0F};
  float body_height{0.0F};
  float yaw_speed{0.0F};
  std::uint8_t motion_mode{0U};
  float progress{0.0F};
  std::uint8_t gait_type{0U};
  bool battery_present{false};
  float battery_percentage{0.0F};
  float battery_voltage{0.0F};
  float battery_current{0.0F};
  bool battery_charging{false};
};

std::string encode_string(const std::string & value);
std::string decode_string(const std::string & value);

bool parse_line(const std::string & line, std::string * type, Fields * fields);

std::string encode_control_command(const ControlCommand & command);
std::string encode_stop_command(const StopCommand & command);
std::string encode_motion_command(const MotionCommand & command);
std::string encode_light_command(const LightCommand & command);
std::string encode_ack(const Ack & ack);
std::string encode_health_status(const HealthStatus & health);
std::string encode_state_stream(const StateStream & state);
std::string encode_health_request();
std::string encode_state_subscribe();

bool decode_control_command(const std::string & line, ControlCommand * command);
bool decode_stop_command(const std::string & line, StopCommand * command);
bool decode_motion_command(const std::string & line, MotionCommand * command);
bool decode_light_command(const std::string & line, LightCommand * command);
bool decode_ack(const std::string & line, Ack * ack);
bool decode_health_status(const std::string & line, HealthStatus * health);
bool decode_state_stream(const std::string & line, StateStream * state);

}  // namespace a2_unitree_ipc

#endif  // A2_UNITREE_IPC_PROTOCOL_HPP_
