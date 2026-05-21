#include "a2_unitree_ipc/protocol.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <charconv>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace a2_unitree_ipc
{
namespace
{

template<typename T>
std::string number_string(T value)
{
  std::ostringstream out;
  out << std::setprecision(9) << value;
  return out.str();
}

std::string bool_string(bool value)
{
  return value ? "1" : "0";
}

bool parse_bool(const Fields & fields, const std::string & key, bool default_value = false)
{
  const auto it = fields.find(key);
  if (it == fields.end()) {
    return default_value;
  }
  const std::string value = it->second;
  return value == "1" || value == "true" || value == "True" || value == "yes" || value == "on";
}

std::string parse_string(const Fields & fields, const std::string & key, const std::string & default_value = "")
{
  const auto it = fields.find(key);
  if (it == fields.end()) {
    return default_value;
  }
  return decode_string(it->second);
}

template<typename T>
T parse_number(const Fields & fields, const std::string & key, T default_value)
{
  const auto it = fields.find(key);
  if (it == fields.end()) {
    return default_value;
  }
  std::istringstream in(it->second);
  T value{};
  in >> value;
  return in.fail() ? default_value : value;
}

template<std::size_t N>
std::string encode_array(const std::array<float, N> & values)
{
  std::ostringstream out;
  out << std::setprecision(9);
  for (std::size_t index = 0; index < N; ++index) {
    if (index > 0) {
      out << ",";
    }
    out << values[index];
  }
  return out.str();
}

template<std::size_t N>
std::array<float, N> parse_array(
  const Fields & fields,
  const std::string & key,
  const std::array<float, N> & default_value)
{
  const auto it = fields.find(key);
  if (it == fields.end()) {
    return default_value;
  }

  std::array<float, N> out = default_value;
  std::istringstream in(it->second);
  std::string token;
  std::size_t index = 0;
  while (std::getline(in, token, ',') && index < N) {
    std::istringstream value_in(token);
    value_in >> out[index];
    if (value_in.fail()) {
      out[index] = default_value[index];
    }
    ++index;
  }
  return out;
}

std::string with_fields(const std::string & type, const std::vector<std::pair<std::string, std::string>> & fields)
{
  std::ostringstream out;
  out << type;
  for (const auto & field : fields) {
    out << " " << field.first << "=" << field.second;
  }
  return out.str();
}

bool parse_typed_line(const std::string & line, const std::string & expected, Fields * fields)
{
  std::string type;
  Fields parsed;
  if (!parse_line(line, &type, &parsed)) {
    return false;
  }
  if (type != expected) {
    return false;
  }
  if (fields) {
    *fields = std::move(parsed);
  }
  return true;
}

}  // namespace

std::string encode_string(const std::string & value)
{
  std::ostringstream out;
  out << std::uppercase << std::hex << std::setfill('0');
  for (const unsigned char ch : value) {
    if (std::isalnum(ch) || ch == '_' || ch == '-' || ch == '.' || ch == ':' || ch == '/') {
      out << static_cast<char>(ch);
    } else {
      out << '%' << std::setw(2) << static_cast<int>(ch);
    }
  }
  return out.str();
}

std::string decode_string(const std::string & value)
{
  std::string out;
  out.reserve(value.size());
  for (std::size_t index = 0; index < value.size(); ++index) {
    if (value[index] == '%' && index + 2 < value.size()) {
      const std::string hex = value.substr(index + 1, 2);
      int byte = 0;
      std::istringstream in(hex);
      in >> std::hex >> byte;
      if (!in.fail()) {
        out.push_back(static_cast<char>(byte));
        index += 2;
        continue;
      }
    }
    out.push_back(value[index]);
  }
  return out;
}

bool parse_line(const std::string & line, std::string * type, Fields * fields)
{
  std::istringstream in(line);
  std::string parsed_type;
  if (!(in >> parsed_type)) {
    return false;
  }

  Fields parsed_fields;
  std::string token;
  while (in >> token) {
    const auto separator = token.find('=');
    if (separator == std::string::npos || separator == 0) {
      continue;
    }
    parsed_fields[token.substr(0, separator)] = token.substr(separator + 1);
  }

  if (type) {
    *type = parsed_type;
  }
  if (fields) {
    *fields = std::move(parsed_fields);
  }
  return true;
}

std::string encode_control_command(const ControlCommand & command)
{
  return with_fields("CONTROL", {
    {"seq", std::to_string(command.seq)},
    {"vx", number_string(command.linear_x)},
    {"vy", number_string(command.linear_y)},
    {"wz", number_string(command.angular_z)},
    {"timeout_ms", std::to_string(command.timeout_ms)},
    {"gait_type", std::to_string(command.gait_type)},
    {"speed_level", std::to_string(command.speed_level)},
    {"body_height", number_string(command.body_height)},
    {"auto_recovery", bool_string(command.auto_recovery)},
  });
}

std::string encode_stop_command(const StopCommand & command)
{
  return with_fields("STOP", {
    {"seq", std::to_string(command.seq)},
    {"reason", encode_string(command.reason)},
  });
}

std::string encode_motion_command(const MotionCommand & command)
{
  return with_fields("MOTION", {
    {"seq", std::to_string(command.seq)},
    {"command", encode_string(command.command)},
    {"int_value", std::to_string(command.int_value)},
    {"float_value", number_string(command.float_value)},
    {"bool_value", bool_string(command.bool_value)},
  });
}

std::string encode_light_command(const LightCommand & command)
{
  return with_fields("LIGHT", {
    {"seq", std::to_string(command.seq)},
    {"on", bool_string(command.on)},
    {"color_mode", std::to_string(command.color_mode)},
    {"intensity", std::to_string(command.intensity)},
    {"r", std::to_string(command.r)},
    {"g", std::to_string(command.g)},
    {"b", std::to_string(command.b)},
    {"ct", std::to_string(command.color_temperature_kelvin)},
  });
}

std::string encode_ack(const Ack & ack)
{
  return with_fields("ACK", {
    {"seq", std::to_string(ack.seq)},
    {"ok", bool_string(ack.ok)},
    {"code", std::to_string(ack.code)},
    {"message", encode_string(ack.message)},
  });
}

std::string encode_health_status(const HealthStatus & health)
{
  return with_fields("HEALTH_STATUS", {
    {"connected", bool_string(health.connected)},
    {"sdk_ready", bool_string(health.sdk_ready)},
    {"ipc_ready", bool_string(health.ipc_ready)},
    {"state", encode_string(health.state)},
    {"reason", encode_string(health.reason)},
    {"last_stop_reason", encode_string(health.last_stop_reason)},
  });
}

std::string encode_state_stream(const StateStream & state)
{
  return with_fields("STATE", {
    {"seq", std::to_string(state.seq)},
    {"source", encode_string(state.source_mode)},
    {"connected", bool_string(state.connected)},
    {"imu_valid", bool_string(state.imu_valid)},
    {"odom_valid", bool_string(state.odom_valid)},
    {"position", encode_array(state.position)},
    {"velocity", encode_array(state.velocity)},
    {"orientation", encode_array(state.orientation_xyzw)},
    {"rpy", encode_array(state.rpy)},
    {"accel", encode_array(state.linear_acceleration)},
    {"gyro", encode_array(state.angular_velocity)},
    {"body_height", number_string(state.body_height)},
    {"yaw_speed", number_string(state.yaw_speed)},
    {"motion_mode", std::to_string(state.motion_mode)},
    {"progress", number_string(state.progress)},
    {"gait_type", std::to_string(state.gait_type)},
    {"battery_present", bool_string(state.battery_present)},
    {"battery_pct", number_string(state.battery_percentage)},
    {"battery_voltage", number_string(state.battery_voltage)},
    {"battery_current", number_string(state.battery_current)},
    {"battery_charging", bool_string(state.battery_charging)},
  });
}

std::string encode_health_request()
{
  return "HEALTH";
}

std::string encode_state_subscribe()
{
  return "SUBSCRIBE_STATE";
}

bool decode_control_command(const std::string & line, ControlCommand * command)
{
  Fields fields;
  if (!parse_typed_line(line, "CONTROL", &fields) || command == nullptr) {
    return false;
  }
  command->seq = parse_number<std::uint64_t>(fields, "seq", 0U);
  command->linear_x = parse_number<double>(fields, "vx", 0.0);
  command->linear_y = parse_number<double>(fields, "vy", 0.0);
  command->angular_z = parse_number<double>(fields, "wz", 0.0);
  command->timeout_ms = parse_number<int>(fields, "timeout_ms", 300);
  command->gait_type = parse_number<int>(fields, "gait_type", 1);
  command->speed_level = parse_number<int>(fields, "speed_level", 1);
  command->body_height = parse_number<double>(fields, "body_height", 0.0);
  command->auto_recovery = parse_bool(fields, "auto_recovery", false);
  return true;
}

bool decode_stop_command(const std::string & line, StopCommand * command)
{
  Fields fields;
  if (!parse_typed_line(line, "STOP", &fields) || command == nullptr) {
    return false;
  }
  command->seq = parse_number<std::uint64_t>(fields, "seq", 0U);
  command->reason = parse_string(fields, "reason", "unspecified");
  return true;
}

bool decode_motion_command(const std::string & line, MotionCommand * command)
{
  Fields fields;
  if (!parse_typed_line(line, "MOTION", &fields) || command == nullptr) {
    return false;
  }
  command->seq = parse_number<std::uint64_t>(fields, "seq", 0U);
  command->command = parse_string(fields, "command", "");
  command->int_value = parse_number<int>(fields, "int_value", 0);
  command->float_value = parse_number<double>(fields, "float_value", 0.0);
  command->bool_value = parse_bool(fields, "bool_value", false);
  return true;
}

bool decode_light_command(const std::string & line, LightCommand * command)
{
  Fields fields;
  if (!parse_typed_line(line, "LIGHT", &fields) || command == nullptr) {
    return false;
  }
  command->seq = parse_number<std::uint64_t>(fields, "seq", 0U);
  command->on = parse_bool(fields, "on", false);
  command->color_mode = parse_number<int>(fields, "color_mode", 0);
  command->intensity = parse_number<int>(fields, "intensity", 0);
  command->r = parse_number<int>(fields, "r", 0);
  command->g = parse_number<int>(fields, "g", 0);
  command->b = parse_number<int>(fields, "b", 0);
  command->color_temperature_kelvin = parse_number<int>(fields, "ct", 4500);
  return true;
}

bool decode_ack(const std::string & line, Ack * ack)
{
  Fields fields;
  if (!parse_typed_line(line, "ACK", &fields) || ack == nullptr) {
    return false;
  }
  ack->seq = parse_number<std::uint64_t>(fields, "seq", 0U);
  ack->ok = parse_bool(fields, "ok", false);
  ack->code = parse_number<int>(fields, "code", 0);
  ack->message = parse_string(fields, "message", "");
  return true;
}

bool decode_health_status(const std::string & line, HealthStatus * health)
{
  Fields fields;
  if (!parse_typed_line(line, "HEALTH_STATUS", &fields) || health == nullptr) {
    return false;
  }
  health->connected = parse_bool(fields, "connected", false);
  health->sdk_ready = parse_bool(fields, "sdk_ready", false);
  health->ipc_ready = parse_bool(fields, "ipc_ready", false);
  health->state = parse_string(fields, "state", "unknown");
  health->reason = parse_string(fields, "reason", "unknown");
  health->last_stop_reason = parse_string(fields, "last_stop_reason", "none");
  return true;
}

bool decode_state_stream(const std::string & line, StateStream * state)
{
  Fields fields;
  if (!parse_typed_line(line, "STATE", &fields) || state == nullptr) {
    return false;
  }
  state->seq = parse_number<std::uint64_t>(fields, "seq", 0U);
  state->source_mode = parse_string(fields, "source", "unknown");
  state->connected = parse_bool(fields, "connected", false);
  state->imu_valid = parse_bool(fields, "imu_valid", false);
  state->odom_valid = parse_bool(fields, "odom_valid", false);
  state->position = parse_array<3>(fields, "position", state->position);
  state->velocity = parse_array<3>(fields, "velocity", state->velocity);
  state->orientation_xyzw = parse_array<4>(fields, "orientation", state->orientation_xyzw);
  state->rpy = parse_array<3>(fields, "rpy", state->rpy);
  state->linear_acceleration = parse_array<3>(fields, "accel", state->linear_acceleration);
  state->angular_velocity = parse_array<3>(fields, "gyro", state->angular_velocity);
  state->body_height = parse_number<float>(fields, "body_height", 0.0F);
  state->yaw_speed = parse_number<float>(fields, "yaw_speed", 0.0F);
  state->motion_mode = static_cast<std::uint8_t>(parse_number<int>(fields, "motion_mode", 0));
  state->progress = parse_number<float>(fields, "progress", 0.0F);
  state->gait_type = static_cast<std::uint8_t>(parse_number<int>(fields, "gait_type", 0));
  state->battery_present = parse_bool(fields, "battery_present", false);
  state->battery_percentage = parse_number<float>(fields, "battery_pct", 0.0F);
  state->battery_voltage = parse_number<float>(fields, "battery_voltage", 0.0F);
  state->battery_current = parse_number<float>(fields, "battery_current", 0.0F);
  state->battery_charging = parse_bool(fields, "battery_charging", false);
  return true;
}

}  // namespace a2_unitree_ipc
