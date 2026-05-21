#include "a2_unitree_ipc/client.hpp"

#include "a2_unitree_ipc/protocol.hpp"

#include <cerrno>
#include <cstring>
#include <utility>

#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace a2_unitree_ipc
{
namespace
{

std::string errno_message(const std::string & prefix)
{
  return prefix + ": " + std::strerror(errno);
}

void set_error(std::string * error_message, const std::string & value)
{
  if (error_message) {
    *error_message = value;
  }
}

}  // namespace

UnixSocketClient::UnixSocketClient(std::string socket_path, int timeout_ms)
: socket_path_(std::move(socket_path)), timeout_ms_(timeout_ms)
{
}

UnixSocketClient::~UnixSocketClient()
{
  close();
}

bool UnixSocketClient::connect_once(std::string * error_message)
{
  std::lock_guard<std::mutex> guard(mutex_);
  if (fd_ >= 0) {
    return true;
  }

  int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
  if (fd < 0) {
    set_error(error_message, errno_message("socket"));
    return false;
  }

  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  if (socket_path_.size() >= sizeof(addr.sun_path)) {
    set_error(error_message, "socket path is too long: " + socket_path_);
    ::close(fd);
    return false;
  }
  std::strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);

  if (::connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
    set_error(error_message, errno_message("connect " + socket_path_));
    ::close(fd);
    return false;
  }

  fd_ = fd;
  read_buffer_.clear();
  return true;
}

bool UnixSocketClient::ensure_connected(std::string * error_message)
{
  {
    std::lock_guard<std::mutex> guard(mutex_);
    if (fd_ >= 0) {
      return true;
    }
  }
  return connect_once(error_message);
}

void UnixSocketClient::close()
{
  std::lock_guard<std::mutex> guard(mutex_);
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  read_buffer_.clear();
}

bool UnixSocketClient::connected() const
{
  std::lock_guard<std::mutex> guard(mutex_);
  return fd_ >= 0;
}

bool UnixSocketClient::send_message(const std::string & message, std::string * error_message)
{
  if (!ensure_connected(error_message)) {
    return false;
  }

  std::string payload;
  if (!encode_frame(message, &payload, error_message)) {
    return false;
  }

  std::lock_guard<std::mutex> guard(mutex_);
  const char * data = payload.data();
  std::size_t remaining = payload.size();
  while (remaining > 0) {
    const auto written = ::send(fd_, data, remaining, MSG_NOSIGNAL);
    if (written < 0) {
      set_error(error_message, errno_message("send"));
      ::close(fd_);
      fd_ = -1;
      return false;
    }
    if (written == 0) {
      set_error(error_message, "send returned zero bytes");
      ::close(fd_);
      fd_ = -1;
      return false;
    }
    data += written;
    remaining -= static_cast<std::size_t>(written);
  }
  return true;
}

bool UnixSocketClient::read_message(std::string * message, int timeout_ms, std::string * error_message)
{
  if (!message) {
    set_error(error_message, "read_message called with null output");
    return false;
  }
  if (!ensure_connected(error_message)) {
    return false;
  }

  std::lock_guard<std::mutex> guard(mutex_);
  while (true) {
    const FrameDecodeStatus status = try_decode_frame(&read_buffer_, message, error_message);
    if (status == FrameDecodeStatus::kReady) {
      return true;
    }
    if (status == FrameDecodeStatus::kError) {
      ::close(fd_);
      fd_ = -1;
      return false;
    }

    pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLIN;
    const int rc = ::poll(&pfd, 1, timeout_ms >= 0 ? timeout_ms : timeout_ms_);
    if (rc == 0) {
      set_error(error_message, "read timeout");
      return false;
    }
    if (rc < 0) {
      set_error(error_message, errno_message("poll"));
      return false;
    }
    if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      set_error(error_message, "socket closed by peer");
      ::close(fd_);
      fd_ = -1;
      return false;
    }

    char buffer[1024];
    const auto received = ::recv(fd_, buffer, sizeof(buffer), 0);
    if (received < 0) {
      set_error(error_message, errno_message("recv"));
      ::close(fd_);
      fd_ = -1;
      return false;
    }
    if (received == 0) {
      set_error(error_message, "socket closed by peer");
      ::close(fd_);
      fd_ = -1;
      return false;
    }
    read_buffer_.append(buffer, static_cast<std::size_t>(received));
  }
}

}  // namespace a2_unitree_ipc
