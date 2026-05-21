#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <csignal>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include "a2_unitree_ipc/protocol.hpp"

#if A2_ENABLE_UNITREE_SDK
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/robot/a2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#endif

namespace
{

using Clock = std::chrono::steady_clock;

std::atomic_bool g_shutdown_requested{false};

void signal_handler(int)
{
  g_shutdown_requested.store(true);
}

std::string errno_string(const std::string & prefix)
{
  return prefix + ": " + std::strerror(errno);
}

bool set_nonblocking(int fd)
{
  const int flags = ::fcntl(fd, F_GETFL, 0);
  if (flags < 0) {
    return false;
  }
  return ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

bool is_zero_motion(const a2_unitree_ipc::ControlCommand & command)
{
  return std::abs(command.linear_x) < 1e-4 &&
    std::abs(command.linear_y) < 1e-4 &&
    std::abs(command.angular_z) < 1e-4;
}

int clamp_int(int value, int minimum, int maximum)
{
  return std::max(minimum, std::min(maximum, value));
}

class UnitreeSdkFacade
{
public:
  bool init(const std::string & interface_name, int dds_domain_id)
  {
    interface_name_ = interface_name;
    dds_domain_id_ = dds_domain_id;
#if A2_ENABLE_UNITREE_SDK
    try {
      unitree::robot::ChannelFactory::Instance()->Init(dds_domain_id_, interface_name_);
      sport_client_ = std::make_unique<unitree::robot::a2::SportClient>();
      sport_client_->SetTimeout(25.0F);
      sport_client_->Init();
      sport_sub_ = std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(
        "rt/lf/sportmodestate");
      sport_sub_->InitChannel(std::bind(&UnitreeSdkFacade::on_sport_state, this, std::placeholders::_1), 1);

      for (const auto & topic : std::vector<std::string>{
          "rt/lf/lowstate", "rt/lowstate", "lf/lowstate", "lowstate"}) {
        auto sub = std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>>(topic);
        sub->InitChannel(std::bind(&UnitreeSdkFacade::on_low_state, this, std::placeholders::_1), 1);
        low_subs_.push_back(sub);
      }
      for (const auto & topic : std::vector<std::string>{
          "lf/bmsstate", "rt/lf/bmsstate", "rt/bmsstate", "bmsstate"}) {
        auto sub = std::make_shared<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::BmsState_>>(topic);
        sub->InitChannel(std::bind(&UnitreeSdkFacade::on_bms_state, this, std::placeholders::_1), 1);
        bms_subs_.push_back(sub);
      }

      {
        std::lock_guard<std::mutex> guard(mutex_);
        health_.connected = true;
        health_.sdk_ready = true;
        health_.ipc_ready = true;
        health_.state = "ready";
        health_.reason = "sdk_initialized";
        state_.source_mode = "real";
        state_.connected = true;
      }
      std::cerr << "[unitree_agent] Unitree SDK initialized on interface=" << interface_name_
                << " dds_domain_id=" << dds_domain_id_ << "\n";
      return true;
    } catch (const std::exception & exc) {
      std::lock_guard<std::mutex> guard(mutex_);
      health_.connected = false;
      health_.sdk_ready = false;
      health_.ipc_ready = true;
      health_.state = "error";
      health_.reason = std::string("sdk_init_exception:") + exc.what();
      std::cerr << "[unitree_agent][safety_stop] reason=sdk_init_exception detail=" << exc.what() << "\n";
      return false;
    }
#else
    {
      std::lock_guard<std::mutex> guard(mutex_);
      health_.connected = true;
      health_.sdk_ready = false;
      health_.ipc_ready = true;
      health_.state = "mock";
      health_.reason = "built_without_unitree_sdk2";
      state_.source_mode = "mock";
      state_.connected = true;
      state_.imu_valid = true;
      state_.odom_valid = true;
      state_.position = {0.0F, 0.0F, 0.28F};
      state_.orientation_xyzw = {0.0F, 0.0F, 0.0F, 1.0F};
      state_.linear_acceleration = {0.0F, 0.0F, 9.81F};
      state_.battery_present = true;
      state_.battery_percentage = 0.85F;
      state_.battery_voltage = 29.4F;
      state_.motion_mode = 1U;
      state_.gait_type = 1U;
    }
    std::cerr << "[unitree_agent] built without Unitree SDK2; running mock IPC agent on interface="
              << interface_name_ << " dds_domain_id=" << dds_domain_id_ << "\n";
    return true;
#endif
  }

  a2_unitree_ipc::Ack handle_control(const a2_unitree_ipc::ControlCommand & command)
  {
    a2_unitree_ipc::Ack ack;
    ack.seq = command.seq;
#if A2_ENABLE_UNITREE_SDK
    if (!sport_client_) {
      ack.ok = false;
      ack.code = -101;
      ack.message = "sport_client_unavailable";
      safety_stop("sdk_unavailable");
      return ack;
    }
    try {
      apply_motion_options(command);
      int code = 0;
      if (is_zero_motion(command)) {
        code = sport_client_->StopMove();
      } else {
        code = sport_client_->Move(
          static_cast<float>(command.linear_x),
          static_cast<float>(command.linear_y),
          static_cast<float>(command.angular_z));
      }
      ack.ok = code == 0;
      ack.code = code;
      ack.message = ack.ok ? "ok" : "move_failed";
      if (!ack.ok) {
        safety_stop("sdk_move_failed");
      }
      return ack;
    } catch (const std::exception & exc) {
      ack.ok = false;
      ack.code = -102;
      ack.message = std::string("sdk_exception:") + exc.what();
      safety_stop("sdk_exception");
      return ack;
    }
#else
    {
      std::lock_guard<std::mutex> guard(mutex_);
      state_.seq += 1;
      state_.velocity[0] = static_cast<float>(command.linear_x);
      state_.velocity[1] = static_cast<float>(command.linear_y);
      state_.angular_velocity[2] = static_cast<float>(command.angular_z);
      state_.yaw_speed = static_cast<float>(command.angular_z);
      state_.gait_type = static_cast<std::uint8_t>(clamp_int(command.gait_type, 0, 255));
      state_.body_height = static_cast<float>(command.body_height);
      health_.state = is_zero_motion(command) ? "idle" : "ready";
      health_.reason = is_zero_motion(command) ? "stop_command" : "control_command";
    }
    ack.ok = true;
    ack.code = 0;
    ack.message = "mock_ok";
    return ack;
#endif
  }

  a2_unitree_ipc::Ack handle_stop(const a2_unitree_ipc::StopCommand & command)
  {
    a2_unitree_ipc::Ack ack;
    ack.seq = command.seq;
    const int code = safety_stop(command.reason);
    ack.ok = code == 0;
    ack.code = code;
    ack.message = ack.ok ? "stopped" : "stop_failed";
    return ack;
  }

  a2_unitree_ipc::Ack handle_motion(const a2_unitree_ipc::MotionCommand & command)
  {
    a2_unitree_ipc::Ack ack;
    ack.seq = command.seq;
#if A2_ENABLE_UNITREE_SDK
    if (!sport_client_) {
      ack.ok = false;
      ack.code = -101;
      ack.message = "sport_client_unavailable";
      safety_stop("sdk_unavailable");
      return ack;
    }
    int code = -1;
    try {
      if (command.command == "stop") {
        code = sport_client_->StopMove();
      } else if (command.command == "balance_stand") {
        code = sport_client_->BalanceStand();
      } else if (command.command == "stand_up") {
        code = sport_client_->StandUp();
      } else if (command.command == "stand_down") {
        code = sport_client_->StandDown();
      } else if (command.command == "recovery_stand") {
        code = sport_client_->RecoveryStand();
      } else if (command.command == "damp") {
        code = sport_client_->Damp();
      } else if (command.command == "switch_gait") {
        code = sport_client_->SwitchGait(command.int_value);
        last_gait_type_ = command.int_value;
      } else if (command.command == "speed_level") {
        code = sport_client_->SpeedLevel(command.int_value);
        last_speed_level_ = command.int_value;
      } else if (command.command == "body_height") {
        code = sport_client_->BodyHeight(static_cast<float>(command.float_value));
        last_body_height_ = command.float_value;
      } else if (command.command == "set_auto_recovery") {
        code = sport_client_->SetAutoRecovery(command.bool_value ? 1 : 0);
        last_auto_recovery_ = command.bool_value;
      }
      ack.ok = code == 0;
      ack.code = code;
      ack.message = ack.ok ? "ok" : "motion_command_failed";
      if (!ack.ok) {
        safety_stop("sdk_motion_command_failed");
      }
      return ack;
    } catch (const std::exception & exc) {
      ack.ok = false;
      ack.code = -102;
      ack.message = std::string("sdk_exception:") + exc.what();
      safety_stop("sdk_exception");
      return ack;
    }
#else
    {
      std::lock_guard<std::mutex> guard(mutex_);
      health_.state = command.command == "stop" ? "idle" : "mock";
      health_.reason = "mock_motion_" + command.command;
      if (command.command == "switch_gait") {
        state_.gait_type = static_cast<std::uint8_t>(clamp_int(command.int_value, 0, 255));
      } else if (command.command == "body_height") {
        state_.body_height = static_cast<float>(command.float_value);
      }
    }
    ack.ok = true;
    ack.code = 0;
    ack.message = "mock_ok";
    return ack;
#endif
  }

  a2_unitree_ipc::Ack handle_light(const a2_unitree_ipc::LightCommand & command)
  {
    a2_unitree_ipc::Ack ack;
    ack.seq = command.seq;
    ack.ok = true;
    ack.code = 0;
    ack.message = "accepted";
    (void)command;
    return ack;
  }

  int safety_stop(const std::string & reason)
  {
    {
      std::lock_guard<std::mutex> guard(mutex_);
      health_.last_stop_reason = reason;
      health_.state = "safe_stop";
      health_.reason = reason;
      state_.velocity = {0.0F, 0.0F, 0.0F};
      state_.angular_velocity[2] = 0.0F;
      state_.yaw_speed = 0.0F;
    }
    if (reason != last_logged_stop_reason_) {
      std::cerr << "[unitree_agent][safety_stop] reason=" << reason << "\n";
      last_logged_stop_reason_ = reason;
    }
#if A2_ENABLE_UNITREE_SDK
    if (sport_client_) {
      try {
        return sport_client_->StopMove();
      } catch (const std::exception & exc) {
        std::cerr << "[unitree_agent][safety_stop] reason=sdk_stop_exception detail=" << exc.what() << "\n";
        return -102;
      }
    }
    return -101;
#else
    return 0;
#endif
  }

  a2_unitree_ipc::StateStream state()
  {
    std::lock_guard<std::mutex> guard(mutex_);
    state_.seq += 1;
    return state_;
  }

  a2_unitree_ipc::HealthStatus health() const
  {
    std::lock_guard<std::mutex> guard(mutex_);
    return health_;
  }

private:
#if A2_ENABLE_UNITREE_SDK
  void apply_motion_options(const a2_unitree_ipc::ControlCommand & command)
  {
    if (command.gait_type != last_gait_type_) {
      const int code = sport_client_->SwitchGait(command.gait_type);
      if (code == 0) {
        last_gait_type_ = command.gait_type;
      }
    }
    if (command.speed_level != last_speed_level_) {
      const int code = sport_client_->SpeedLevel(command.speed_level);
      if (code == 0) {
        last_speed_level_ = command.speed_level;
      }
    }
    if (std::abs(command.body_height - last_body_height_) > 1e-4) {
      const int code = sport_client_->BodyHeight(static_cast<float>(command.body_height));
      if (code == 0) {
        last_body_height_ = command.body_height;
      }
    }
    if (command.auto_recovery != last_auto_recovery_) {
      const int code = sport_client_->SetAutoRecovery(command.auto_recovery ? 1 : 0);
      if (code == 0) {
        last_auto_recovery_ = command.auto_recovery;
      }
    }
  }

  void on_sport_state(const void * message)
  {
    const auto & sdk_state = *static_cast<const unitree_go::msg::dds_::SportModeState_ *>(message);
    std::lock_guard<std::mutex> guard(mutex_);
    state_.source_mode = "real";
    state_.connected = true;
    state_.imu_valid = true;
    state_.odom_valid = true;
    for (std::size_t index = 0; index < 3U; ++index) {
      state_.position[index] = sdk_state.position()[index];
      state_.velocity[index] = sdk_state.velocity()[index];
      state_.rpy[index] = sdk_state.imu_state().rpy()[index];
      state_.linear_acceleration[index] = sdk_state.imu_state().accelerometer()[index];
      state_.angular_velocity[index] = sdk_state.imu_state().gyroscope()[index];
    }
    for (std::size_t index = 0; index < 4U; ++index) {
      state_.orientation_xyzw[index] = sdk_state.imu_state().quaternion()[index];
    }
    state_.body_height = sdk_state.body_height();
    state_.yaw_speed = sdk_state.yaw_speed();
    state_.motion_mode = sdk_state.mode();
    state_.progress = sdk_state.progress();
    state_.gait_type = sdk_state.gait_type();
    health_.connected = true;
    health_.sdk_ready = true;
    health_.state = "ready";
    health_.reason = "a2_state_ok";
  }

  void on_low_state(const void * message)
  {
    const auto & sdk_state = *static_cast<const unitree_go::msg::dds_::LowState_ *>(message);
    std::lock_guard<std::mutex> guard(mutex_);
    state_.battery_present = true;
    state_.battery_percentage = std::max(0.0F, std::min(1.0F, static_cast<float>(sdk_state.bms_state().soc()) / 100.0F));
    state_.battery_voltage = sdk_state.power_v();
    state_.battery_current = sdk_state.power_a();
    state_.battery_charging = state_.battery_current < -0.1F;
  }

  void on_bms_state(const void * message)
  {
    const auto & sdk_state = *static_cast<const unitree_hg::msg::dds_::BmsState_ *>(message);
    std::lock_guard<std::mutex> guard(mutex_);
    state_.battery_present = true;
    state_.battery_percentage = std::max(0.0F, std::min(1.0F, static_cast<float>(sdk_state.soc()) / 100.0F));
    float voltage = 0.0F;
    const auto & bms_voltage = sdk_state.bmsvoltage();
    if (bms_voltage[0] > 0) {
      voltage = static_cast<float>(bms_voltage[0]) / 1000.0F;
    } else {
      float sum_mv = 0.0F;
      for (const auto mv : sdk_state.cell_vol()) {
        sum_mv += static_cast<float>(mv);
      }
      if (sum_mv > 0.0F) {
        voltage = sum_mv / 1000.0F;
      }
    }
    state_.battery_voltage = voltage;
    state_.battery_current = static_cast<float>(sdk_state.current()) / 1000.0F;
    state_.battery_charging = state_.battery_current < -0.1F;
  }
#endif

  std::string interface_name_;
  int dds_domain_id_{0};
  mutable std::mutex mutex_;
  a2_unitree_ipc::StateStream state_;
  a2_unitree_ipc::HealthStatus health_;
  std::string last_logged_stop_reason_;
  int last_gait_type_{1};
  int last_speed_level_{1};
  double last_body_height_{0.0};
  bool last_auto_recovery_{false};

#if A2_ENABLE_UNITREE_SDK
  std::unique_ptr<unitree::robot::a2::SportClient> sport_client_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>> sport_sub_;
  std::vector<std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>>> low_subs_;
  std::vector<std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::BmsState_>>> bms_subs_;
#endif
};

struct Client
{
  int fd{-1};
  std::string buffer;
  bool state_subscriber{false};
  bool control_client{false};
};

class AgentServer
{
public:
  AgentServer(std::string socket_path, UnitreeSdkFacade & sdk, int default_command_timeout_ms)
  : socket_path_(std::move(socket_path)),
    sdk_(sdk),
    default_command_timeout_ms_(default_command_timeout_ms),
    command_timeout_ms_(default_command_timeout_ms)
  {
  }

  ~AgentServer()
  {
    close_all();
  }

  bool start()
  {
    const auto slash = socket_path_.find_last_of('/');
    if (slash != std::string::npos) {
      const std::string dir = socket_path_.substr(0, slash);
      if (!dir.empty() && ::mkdir(dir.c_str(), 0755) != 0 && errno != EEXIST) {
        std::cerr << "[unitree_agent] " << errno_string("mkdir " + dir) << "\n";
        return false;
      }
    }

    ::unlink(socket_path_.c_str());
    server_fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
      std::cerr << "[unitree_agent] " << errno_string("socket") << "\n";
      return false;
    }
    if (!set_nonblocking(server_fd_)) {
      std::cerr << "[unitree_agent] " << errno_string("fcntl nonblock") << "\n";
      return false;
    }

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    if (socket_path_.size() >= sizeof(addr.sun_path)) {
      std::cerr << "[unitree_agent] socket path too long: " << socket_path_ << "\n";
      return false;
    }
    std::strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);
    if (::bind(server_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
      std::cerr << "[unitree_agent] " << errno_string("bind " + socket_path_) << "\n";
      return false;
    }
    ::chmod(socket_path_.c_str(), 0660);
    if (::listen(server_fd_, 16) != 0) {
      std::cerr << "[unitree_agent] " << errno_string("listen") << "\n";
      return false;
    }
    std::cerr << "[unitree_agent] UDS server listening socket=" << socket_path_ << "\n";
    return true;
  }

  void run()
  {
    auto last_state_broadcast = Clock::now();
    while (!g_shutdown_requested.load()) {
      poll_once(50);
      safety_tick();
      const auto now = Clock::now();
      if (now - last_state_broadcast >= std::chrono::milliseconds(100)) {
        broadcast_state();
        last_state_broadcast = now;
      }
    }
    sdk_.safety_stop("process_exit");
  }

private:
  void poll_once(int timeout_ms)
  {
    std::vector<pollfd> fds;
    fds.push_back({server_fd_, POLLIN, 0});
    for (const auto & client : clients_) {
      fds.push_back({client.fd, POLLIN, 0});
    }

    const int rc = ::poll(fds.data(), fds.size(), timeout_ms);
    if (rc < 0) {
      if (errno != EINTR) {
        std::cerr << "[unitree_agent] " << errno_string("poll") << "\n";
      }
      return;
    }
    if (rc == 0) {
      return;
    }
    if ((fds[0].revents & POLLIN) != 0) {
      accept_clients();
    }

    std::vector<std::size_t> closed;
    for (std::size_t index = 0; index < clients_.size(); ++index) {
      const auto revents = fds[index + 1].revents;
      if ((revents & (POLLHUP | POLLERR | POLLNVAL)) != 0) {
        closed.push_back(index);
        continue;
      }
      if ((revents & POLLIN) != 0 && !read_client(index)) {
        closed.push_back(index);
      }
    }
    close_indices(closed);
  }

  void accept_clients()
  {
    while (true) {
      int fd = ::accept(server_fd_, nullptr, nullptr);
      if (fd < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
          std::cerr << "[unitree_agent] " << errno_string("accept") << "\n";
        }
        return;
      }
      if (!set_nonblocking(fd)) {
        std::cerr << "[unitree_agent] " << errno_string("fcntl client nonblock") << "\n";
        ::close(fd);
        continue;
      }
      clients_.push_back(Client{fd});
    }
  }

  bool read_client(std::size_t index)
  {
    char buffer[2048];
    while (true) {
      const auto count = ::recv(clients_[index].fd, buffer, sizeof(buffer), 0);
      if (count < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          return true;
        }
        return false;
      }
      if (count == 0) {
        return false;
      }
      clients_[index].buffer.append(buffer, static_cast<std::size_t>(count));
      std::size_t newline = std::string::npos;
      while ((newline = clients_[index].buffer.find('\n')) != std::string::npos) {
        const std::string line = clients_[index].buffer.substr(0, newline);
        clients_[index].buffer.erase(0, newline + 1);
        handle_line(clients_[index], line);
      }
    }
  }

  void handle_line(Client & client, const std::string & line)
  {
    std::string type;
    a2_unitree_ipc::Fields fields;
    if (!a2_unitree_ipc::parse_line(line, &type, &fields)) {
      send_to(client, a2_unitree_ipc::encode_ack({0, false, -1, "bad_request"}));
      return;
    }

    if (type == "SUBSCRIBE_STATE") {
      client.state_subscriber = true;
      send_to(client, a2_unitree_ipc::encode_health_status(sdk_.health()));
      return;
    }
    if (type == "HEALTH") {
      send_to(client, a2_unitree_ipc::encode_health_status(sdk_.health()));
      return;
    }
    if (type == "CONTROL") {
      a2_unitree_ipc::ControlCommand command;
      if (!a2_unitree_ipc::decode_control_command(line, &command)) {
        send_to(client, a2_unitree_ipc::encode_ack({0, false, -2, "bad_control"}));
        return;
      }
      client.control_client = true;
      last_control_time_ = Clock::now();
      command_active_ = !is_zero_motion(command);
      command_timeout_ms_ = command.timeout_ms > 0 ? command.timeout_ms : default_command_timeout_ms_;
      send_to(client, a2_unitree_ipc::encode_ack(sdk_.handle_control(command)));
      return;
    }
    if (type == "STOP") {
      a2_unitree_ipc::StopCommand command;
      if (!a2_unitree_ipc::decode_stop_command(line, &command)) {
        send_to(client, a2_unitree_ipc::encode_ack({0, false, -2, "bad_stop"}));
        return;
      }
      command_active_ = false;
      send_to(client, a2_unitree_ipc::encode_ack(sdk_.handle_stop(command)));
      return;
    }
    if (type == "MOTION") {
      a2_unitree_ipc::MotionCommand command;
      if (!a2_unitree_ipc::decode_motion_command(line, &command)) {
        send_to(client, a2_unitree_ipc::encode_ack({0, false, -2, "bad_motion"}));
        return;
      }
      if (command.command == "stop") {
        command_active_ = false;
      }
      send_to(client, a2_unitree_ipc::encode_ack(sdk_.handle_motion(command)));
      return;
    }
    if (type == "LIGHT") {
      a2_unitree_ipc::LightCommand command;
      if (!a2_unitree_ipc::decode_light_command(line, &command)) {
        send_to(client, a2_unitree_ipc::encode_ack({0, false, -2, "bad_light"}));
        return;
      }
      send_to(client, a2_unitree_ipc::encode_ack(sdk_.handle_light(command)));
      return;
    }
    send_to(client, a2_unitree_ipc::encode_ack({0, false, -3, "unknown_message_type"}));
  }

  bool send_to(Client & client, const std::string & line)
  {
    const std::string payload = line + "\n";
    const auto count = ::send(client.fd, payload.data(), payload.size(), MSG_NOSIGNAL);
    return count == static_cast<ssize_t>(payload.size());
  }

  void broadcast_state()
  {
    const std::string line = a2_unitree_ipc::encode_state_stream(sdk_.state());
    std::vector<std::size_t> closed;
    for (std::size_t index = 0; index < clients_.size(); ++index) {
      if (!clients_[index].state_subscriber) {
        continue;
      }
      if (!send_to(clients_[index], line)) {
        closed.push_back(index);
      }
    }
    close_indices(closed);
  }

  void safety_tick()
  {
    if (!command_active_) {
      return;
    }
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - last_control_time_);
    if (elapsed.count() > command_timeout_ms_) {
      command_active_ = false;
      sdk_.safety_stop("command_timeout");
    }
  }

  void close_indices(std::vector<std::size_t> indices)
  {
    if (indices.empty()) {
      return;
    }
    std::sort(indices.begin(), indices.end());
    indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
    for (auto it = indices.rbegin(); it != indices.rend(); ++it) {
      const std::size_t index = *it;
      if (index >= clients_.size()) {
        continue;
      }
      if (clients_[index].control_client) {
        command_active_ = false;
        sdk_.safety_stop("ipc_disconnect");
      }
      ::close(clients_[index].fd);
      clients_.erase(clients_.begin() + static_cast<std::ptrdiff_t>(index));
    }
  }

  void close_all()
  {
    for (auto & client : clients_) {
      if (client.fd >= 0) {
        ::close(client.fd);
        client.fd = -1;
      }
    }
    clients_.clear();
    if (server_fd_ >= 0) {
      ::close(server_fd_);
      server_fd_ = -1;
    }
    if (!socket_path_.empty()) {
      ::unlink(socket_path_.c_str());
    }
  }

  std::string socket_path_;
  UnitreeSdkFacade & sdk_;
  int default_command_timeout_ms_{300};
  int command_timeout_ms_{300};
  int server_fd_{-1};
  bool command_active_{false};
  Clock::time_point last_control_time_{Clock::now()};
  std::vector<Client> clients_;
};

struct Options
{
  std::string socket_path{a2_unitree_ipc::kDefaultSocketPath};
  std::string interface_name{"eth0"};
  int dds_domain_id{0};
  int command_timeout_ms{300};
};

Options parse_args(int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string arg = argv[index];
    auto require_value = [&](const std::string & name) -> std::string {
      if (index + 1 >= argc) {
        std::cerr << "missing value for " << name << "\n";
        std::exit(2);
      }
      return argv[++index];
    };
    if (arg == "--socket") {
      options.socket_path = require_value(arg);
    } else if (arg == "--interface") {
      options.interface_name = require_value(arg);
    } else if (arg == "--dds-domain-id") {
      options.dds_domain_id = std::max(0, std::stoi(require_value(arg)));
    } else if (arg == "--command-timeout-ms") {
      options.command_timeout_ms = std::max(50, std::stoi(require_value(arg)));
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "usage: unitree_agent --socket /run/a2/unitree_agent.sock --interface eth0 --dds-domain-id 0\n";
      std::exit(0);
    } else {
      std::cerr << "unknown argument: " << arg << "\n";
      std::exit(2);
    }
  }
  return options;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  const Options options = parse_args(argc, argv);
  UnitreeSdkFacade sdk;
  sdk.init(options.interface_name, options.dds_domain_id);

  AgentServer server(options.socket_path, sdk, options.command_timeout_ms);
  if (!server.start()) {
    sdk.safety_stop("ipc_server_start_failed");
    return 2;
  }
  server.run();
  return 0;
}
