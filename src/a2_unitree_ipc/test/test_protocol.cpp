#include <gtest/gtest.h>

#include "a2_unitree_ipc/protocol.hpp"

TEST(UnitreeIpcProtocol, ControlCommandRoundTrip)
{
  a2_unitree_ipc::ControlCommand command;
  command.seq = 42;
  command.linear_x = 0.25;
  command.linear_y = -0.1;
  command.angular_z = 0.35;
  command.timeout_ms = 300;
  command.gait_type = 3;
  command.speed_level = 2;
  command.body_height = 0.05;
  command.auto_recovery = true;

  a2_unitree_ipc::ControlCommand decoded;
  const auto message = a2_unitree_ipc::encode_control_command(command);
  EXPECT_EQ(a2_unitree_ipc::message_type(message), a2_unitree_ipc::MessageType::kControl);
  ASSERT_TRUE(a2_unitree_ipc::decode_control_command(message, &decoded));
  EXPECT_EQ(decoded.seq, 42U);
  EXPECT_DOUBLE_EQ(decoded.linear_x, 0.25);
  EXPECT_DOUBLE_EQ(decoded.linear_y, -0.1);
  EXPECT_DOUBLE_EQ(decoded.angular_z, 0.35);
  EXPECT_EQ(decoded.timeout_ms, 300);
  EXPECT_EQ(decoded.gait_type, 3);
  EXPECT_EQ(decoded.speed_level, 2);
  EXPECT_DOUBLE_EQ(decoded.body_height, 0.05);
  EXPECT_TRUE(decoded.auto_recovery);
}

TEST(UnitreeIpcProtocol, ProtobufFramesUseLengthPrefix)
{
  const auto message = a2_unitree_ipc::encode_health_request();
  std::string frame;
  ASSERT_TRUE(a2_unitree_ipc::encode_frame(message, &frame));
  ASSERT_GE(frame.size(), message.size() + 4U);
  EXPECT_EQ(static_cast<unsigned char>(frame[0]), 0U);
  EXPECT_EQ(static_cast<unsigned char>(frame[1]), 0U);
  EXPECT_EQ(static_cast<unsigned char>(frame[2]), 0U);
  EXPECT_EQ(static_cast<unsigned char>(frame[3]), message.size());

  std::string buffer = frame;
  std::string decoded;
  EXPECT_EQ(
    a2_unitree_ipc::try_decode_frame(&buffer, &decoded),
    a2_unitree_ipc::FrameDecodeStatus::kReady);
  EXPECT_TRUE(buffer.empty());
  EXPECT_EQ(decoded, message);
  EXPECT_EQ(a2_unitree_ipc::message_type(decoded), a2_unitree_ipc::MessageType::kHealthRequest);
}

TEST(UnitreeIpcProtocol, StopCommandCarriesSafeReason)
{
  const auto line = a2_unitree_ipc::encode_stop_command({7, "cmd_timeout"});
  a2_unitree_ipc::StopCommand decoded;

  ASSERT_TRUE(a2_unitree_ipc::decode_stop_command(line, &decoded));
  EXPECT_EQ(decoded.seq, 7U);
  EXPECT_EQ(decoded.reason, "cmd_timeout");
}

TEST(UnitreeIpcProtocol, StateStreamRoundTrip)
{
  a2_unitree_ipc::StateStream state;
  state.seq = 9;
  state.source_mode = "real";
  state.connected = true;
  state.imu_valid = true;
  state.odom_valid = true;
  state.position = {1.0F, 2.0F, 0.3F};
  state.velocity = {0.1F, 0.2F, 0.0F};
  state.orientation_xyzw = {0.0F, 0.0F, 0.5F, 0.866F};
  state.rpy = {0.01F, 0.02F, 1.0F};
  state.linear_acceleration = {0.0F, 0.0F, 9.81F};
  state.angular_velocity = {0.0F, 0.0F, 0.2F};
  state.body_height = 0.28F;
  state.yaw_speed = 0.2F;
  state.motion_mode = 1U;
  state.progress = 0.5F;
  state.gait_type = 2U;
  state.battery_present = true;
  state.battery_percentage = 0.82F;
  state.battery_voltage = 29.4F;
  state.battery_current = -1.2F;
  state.battery_charging = true;

  a2_unitree_ipc::StateStream decoded;
  ASSERT_TRUE(a2_unitree_ipc::decode_state_stream(
    a2_unitree_ipc::encode_state_stream(state), &decoded));
  EXPECT_EQ(decoded.seq, 9U);
  EXPECT_EQ(decoded.source_mode, "real");
  EXPECT_TRUE(decoded.connected);
  EXPECT_TRUE(decoded.imu_valid);
  EXPECT_TRUE(decoded.odom_valid);
  EXPECT_FLOAT_EQ(decoded.position[0], 1.0F);
  EXPECT_FLOAT_EQ(decoded.orientation_xyzw[3], 0.866F);
  EXPECT_FLOAT_EQ(decoded.linear_acceleration[2], 9.81F);
  EXPECT_FLOAT_EQ(decoded.battery_percentage, 0.82F);
  EXPECT_TRUE(decoded.battery_charging);
}

TEST(UnitreeIpcProtocol, HealthStatusRoundTrip)
{
  a2_unitree_ipc::HealthStatus health;
  health.connected = true;
  health.sdk_ready = true;
  health.ipc_ready = true;
  health.state = "ready";
  health.reason = "a2_state_ok";
  health.last_stop_reason = "ipc_disconnect";

  a2_unitree_ipc::HealthStatus decoded;
  ASSERT_TRUE(a2_unitree_ipc::decode_health_status(
    a2_unitree_ipc::encode_health_status(health), &decoded));
  EXPECT_TRUE(decoded.connected);
  EXPECT_TRUE(decoded.sdk_ready);
  EXPECT_TRUE(decoded.ipc_ready);
  EXPECT_EQ(decoded.state, "ready");
  EXPECT_EQ(decoded.reason, "a2_state_ok");
  EXPECT_EQ(decoded.last_stop_reason, "ipc_disconnect");
}
