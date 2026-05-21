#include "a2_unitree_ipc/protocol.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <string>

#include "a2/unitree_agent.pb.h"

namespace a2_unitree_ipc
{
namespace
{

namespace pb = ::a2::unitree_agent::v1;

void set_error(std::string * error_message, const std::string & value)
{
  if (error_message) {
    *error_message = value;
  }
}

template<std::size_t N>
void set_vector3(pb::Vector3f * out, const std::array<float, N> & values)
{
  static_assert(N >= 3);
  out->set_x(values[0]);
  out->set_y(values[1]);
  out->set_z(values[2]);
}

std::array<float, 3> read_vector3(const pb::Vector3f & value)
{
  return {value.x(), value.y(), value.z()};
}

void set_quaternion(pb::Quaternionf * out, const std::array<float, 4> & values)
{
  out->set_x(values[0]);
  out->set_y(values[1]);
  out->set_z(values[2]);
  out->set_w(values[3]);
}

std::array<float, 4> read_quaternion(const pb::Quaternionf & value)
{
  return {value.x(), value.y(), value.z(), value.w()};
}

std::string serialize(const pb::Envelope & envelope)
{
  std::string out;
  envelope.SerializeToString(&out);
  return out;
}

bool parse_envelope(const std::string & message, pb::Envelope * envelope)
{
  return envelope != nullptr && envelope->ParseFromString(message);
}

MessageType from_proto_type(pb::Envelope::Type type)
{
  switch (type) {
    case pb::Envelope::CONTROL:
      return MessageType::kControl;
    case pb::Envelope::STOP:
      return MessageType::kStop;
    case pb::Envelope::MOTION:
      return MessageType::kMotion;
    case pb::Envelope::LIGHT:
      return MessageType::kLight;
    case pb::Envelope::ACK:
      return MessageType::kAck;
    case pb::Envelope::HEALTH_STATUS:
      return MessageType::kHealthStatus;
    case pb::Envelope::STATE:
      return MessageType::kState;
    case pb::Envelope::HEALTH_REQUEST:
      return MessageType::kHealthRequest;
    case pb::Envelope::STATE_SUBSCRIBE:
      return MessageType::kStateSubscribe;
    case pb::Envelope::TYPE_UNSPECIFIED:
    default:
      return MessageType::kUnknown;
  }
}

}  // namespace

bool encode_frame(
  const std::string & message,
  std::string * frame,
  std::string * error_message)
{
  if (frame == nullptr) {
    set_error(error_message, "encode_frame called with null output");
    return false;
  }
  if (message.size() > kMaxFrameBytes) {
    set_error(error_message, "message exceeds max protobuf frame size");
    return false;
  }

  const auto length = static_cast<std::uint32_t>(message.size());
  frame->clear();
  frame->reserve(sizeof(length) + message.size());
  frame->push_back(static_cast<char>((length >> 24U) & 0xFFU));
  frame->push_back(static_cast<char>((length >> 16U) & 0xFFU));
  frame->push_back(static_cast<char>((length >> 8U) & 0xFFU));
  frame->push_back(static_cast<char>(length & 0xFFU));
  frame->append(message);
  return true;
}

FrameDecodeStatus try_decode_frame(
  std::string * buffer,
  std::string * message,
  std::string * error_message)
{
  if (buffer == nullptr || message == nullptr) {
    set_error(error_message, "try_decode_frame called with null input");
    return FrameDecodeStatus::kError;
  }
  if (buffer->size() < 4U) {
    return FrameDecodeStatus::kIncomplete;
  }

  const auto length =
    (static_cast<std::uint32_t>(static_cast<unsigned char>((*buffer)[0])) << 24U) |
    (static_cast<std::uint32_t>(static_cast<unsigned char>((*buffer)[1])) << 16U) |
    (static_cast<std::uint32_t>(static_cast<unsigned char>((*buffer)[2])) << 8U) |
    static_cast<std::uint32_t>(static_cast<unsigned char>((*buffer)[3]));

  if (length > kMaxFrameBytes) {
    set_error(error_message, "protobuf frame exceeds max size");
    return FrameDecodeStatus::kError;
  }
  if (buffer->size() < 4U + static_cast<std::size_t>(length)) {
    return FrameDecodeStatus::kIncomplete;
  }

  *message = buffer->substr(4U, length);
  buffer->erase(0, 4U + static_cast<std::size_t>(length));
  return FrameDecodeStatus::kReady;
}

MessageType message_type(const std::string & message)
{
  pb::Envelope envelope;
  if (!parse_envelope(message, &envelope)) {
    return MessageType::kUnknown;
  }
  return from_proto_type(envelope.type());
}

std::string encode_control_command(const ControlCommand & command)
{
  pb::Envelope envelope;
  envelope.set_type(pb::Envelope::CONTROL);
  auto * out = envelope.mutable_control();
  out->set_seq(command.seq);
  out->set_linear_x(command.linear_x);
  out->set_linear_y(command.linear_y);
  out->set_angular_z(command.angular_z);
  out->set_timeout_ms(std::max(command.timeout_ms, 0));
  out->set_gait_type(command.gait_type);
  out->set_speed_level(command.speed_level);
  out->set_body_height(command.body_height);
  out->set_auto_recovery(command.auto_recovery);
  return serialize(envelope);
}

std::string encode_stop_command(const StopCommand & command)
{
  pb::Envelope envelope;
  envelope.set_type(pb::Envelope::STOP);
  auto * out = envelope.mutable_stop();
  out->set_seq(command.seq);
  out->set_reason(command.reason);
  return serialize(envelope);
}

std::string encode_motion_command(const MotionCommand & command)
{
  pb::Envelope envelope;
  envelope.set_type(pb::Envelope::MOTION);
  auto * out = envelope.mutable_motion();
  out->set_seq(command.seq);
  out->set_command(command.command);
  out->set_int_value(command.int_value);
  out->set_float_value(command.float_value);
  out->set_bool_value(command.bool_value);
  return serialize(envelope);
}

std::string encode_light_command(const LightCommand & command)
{
  pb::Envelope envelope;
  envelope.set_type(pb::Envelope::LIGHT);
  auto * out = envelope.mutable_light();
  out->set_seq(command.seq);
  out->set_on(command.on);
  out->set_color_mode(command.color_mode);
  out->set_intensity(command.intensity);
  out->set_r(command.r);
  out->set_g(command.g);
  out->set_b(command.b);
  out->set_color_temperature_kelvin(command.color_temperature_kelvin);
  return serialize(envelope);
}

std::string encode_ack(const Ack & ack)
{
  pb::Envelope envelope;
  envelope.set_type(pb::Envelope::ACK);
  auto * out = envelope.mutable_ack();
  out->set_seq(ack.seq);
  out->set_ok(ack.ok);
  out->set_code(ack.code);
  out->set_message(ack.message);
  return serialize(envelope);
}

std::string encode_health_status(const HealthStatus & health)
{
  pb::Envelope envelope;
  envelope.set_type(pb::Envelope::HEALTH_STATUS);
  auto * out = envelope.mutable_health_status();
  out->set_connected(health.connected);
  out->set_sdk_ready(health.sdk_ready);
  out->set_ipc_ready(health.ipc_ready);
  out->set_state(health.state);
  out->set_reason(health.reason);
  out->set_last_stop_reason(health.last_stop_reason);
  return serialize(envelope);
}

std::string encode_state_stream(const StateStream & state)
{
  pb::Envelope envelope;
  envelope.set_type(pb::Envelope::STATE);
  auto * out = envelope.mutable_state();
  out->set_seq(state.seq);
  out->set_source_mode(state.source_mode);
  out->set_connected(state.connected);
  out->set_imu_valid(state.imu_valid);
  out->set_odom_valid(state.odom_valid);
  set_vector3(out->mutable_position(), state.position);
  set_vector3(out->mutable_velocity(), state.velocity);
  set_quaternion(out->mutable_orientation_xyzw(), state.orientation_xyzw);
  set_vector3(out->mutable_rpy(), state.rpy);
  set_vector3(out->mutable_linear_acceleration(), state.linear_acceleration);
  set_vector3(out->mutable_angular_velocity(), state.angular_velocity);
  out->set_body_height(state.body_height);
  out->set_yaw_speed(state.yaw_speed);
  out->set_motion_mode(state.motion_mode);
  out->set_progress(state.progress);
  out->set_gait_type(state.gait_type);
  out->set_battery_present(state.battery_present);
  out->set_battery_percentage(state.battery_percentage);
  out->set_battery_voltage(state.battery_voltage);
  out->set_battery_current(state.battery_current);
  out->set_battery_charging(state.battery_charging);
  return serialize(envelope);
}

std::string encode_health_request()
{
  pb::Envelope envelope;
  envelope.set_type(pb::Envelope::HEALTH_REQUEST);
  envelope.mutable_health_request();
  return serialize(envelope);
}

std::string encode_state_subscribe()
{
  pb::Envelope envelope;
  envelope.set_type(pb::Envelope::STATE_SUBSCRIBE);
  envelope.mutable_state_subscribe();
  return serialize(envelope);
}

bool decode_control_command(const std::string & message, ControlCommand * command)
{
  pb::Envelope envelope;
  if (!parse_envelope(message, &envelope) || command == nullptr ||
    envelope.type() != pb::Envelope::CONTROL || !envelope.has_control())
  {
    return false;
  }
  const auto & in = envelope.control();
  command->seq = in.seq();
  command->linear_x = in.linear_x();
  command->linear_y = in.linear_y();
  command->angular_z = in.angular_z();
  command->timeout_ms = static_cast<int>(std::min<std::uint32_t>(
    in.timeout_ms(), static_cast<std::uint32_t>(std::numeric_limits<int>::max())));
  command->gait_type = in.gait_type();
  command->speed_level = in.speed_level();
  command->body_height = in.body_height();
  command->auto_recovery = in.auto_recovery();
  return true;
}

bool decode_stop_command(const std::string & message, StopCommand * command)
{
  pb::Envelope envelope;
  if (!parse_envelope(message, &envelope) || command == nullptr ||
    envelope.type() != pb::Envelope::STOP || !envelope.has_stop())
  {
    return false;
  }
  command->seq = envelope.stop().seq();
  command->reason = envelope.stop().reason();
  return true;
}

bool decode_motion_command(const std::string & message, MotionCommand * command)
{
  pb::Envelope envelope;
  if (!parse_envelope(message, &envelope) || command == nullptr ||
    envelope.type() != pb::Envelope::MOTION || !envelope.has_motion())
  {
    return false;
  }
  const auto & in = envelope.motion();
  command->seq = in.seq();
  command->command = in.command();
  command->int_value = in.int_value();
  command->float_value = in.float_value();
  command->bool_value = in.bool_value();
  return true;
}

bool decode_light_command(const std::string & message, LightCommand * command)
{
  pb::Envelope envelope;
  if (!parse_envelope(message, &envelope) || command == nullptr ||
    envelope.type() != pb::Envelope::LIGHT || !envelope.has_light())
  {
    return false;
  }
  const auto & in = envelope.light();
  command->seq = in.seq();
  command->on = in.on();
  command->color_mode = in.color_mode();
  command->intensity = in.intensity();
  command->r = in.r();
  command->g = in.g();
  command->b = in.b();
  command->color_temperature_kelvin = in.color_temperature_kelvin();
  return true;
}

bool decode_ack(const std::string & message, Ack * ack)
{
  pb::Envelope envelope;
  if (!parse_envelope(message, &envelope) || ack == nullptr ||
    envelope.type() != pb::Envelope::ACK || !envelope.has_ack())
  {
    return false;
  }
  const auto & in = envelope.ack();
  ack->seq = in.seq();
  ack->ok = in.ok();
  ack->code = in.code();
  ack->message = in.message();
  return true;
}

bool decode_health_status(const std::string & message, HealthStatus * health)
{
  pb::Envelope envelope;
  if (!parse_envelope(message, &envelope) || health == nullptr ||
    envelope.type() != pb::Envelope::HEALTH_STATUS || !envelope.has_health_status())
  {
    return false;
  }
  const auto & in = envelope.health_status();
  health->connected = in.connected();
  health->sdk_ready = in.sdk_ready();
  health->ipc_ready = in.ipc_ready();
  health->state = in.state();
  health->reason = in.reason();
  health->last_stop_reason = in.last_stop_reason();
  return true;
}

bool decode_state_stream(const std::string & message, StateStream * state)
{
  pb::Envelope envelope;
  if (!parse_envelope(message, &envelope) || state == nullptr ||
    envelope.type() != pb::Envelope::STATE || !envelope.has_state())
  {
    return false;
  }
  const auto & in = envelope.state();
  state->seq = in.seq();
  state->source_mode = in.source_mode();
  state->connected = in.connected();
  state->imu_valid = in.imu_valid();
  state->odom_valid = in.odom_valid();
  state->position = read_vector3(in.position());
  state->velocity = read_vector3(in.velocity());
  state->orientation_xyzw = read_quaternion(in.orientation_xyzw());
  state->rpy = read_vector3(in.rpy());
  state->linear_acceleration = read_vector3(in.linear_acceleration());
  state->angular_velocity = read_vector3(in.angular_velocity());
  state->body_height = in.body_height();
  state->yaw_speed = in.yaw_speed();
  state->motion_mode = static_cast<std::uint8_t>(std::min(in.motion_mode(), 255U));
  state->progress = in.progress();
  state->gait_type = static_cast<std::uint8_t>(std::min(in.gait_type(), 255U));
  state->battery_present = in.battery_present();
  state->battery_percentage = in.battery_percentage();
  state->battery_voltage = in.battery_voltage();
  state->battery_current = in.battery_current();
  state->battery_charging = in.battery_charging();
  return true;
}

}  // namespace a2_unitree_ipc
