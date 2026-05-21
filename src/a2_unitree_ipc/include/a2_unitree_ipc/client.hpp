#ifndef A2_UNITREE_IPC_CLIENT_HPP_
#define A2_UNITREE_IPC_CLIENT_HPP_

#include <mutex>
#include <string>

namespace a2_unitree_ipc
{

class UnixSocketClient
{
public:
  explicit UnixSocketClient(std::string socket_path, int timeout_ms = 200);
  ~UnixSocketClient();

  UnixSocketClient(const UnixSocketClient &) = delete;
  UnixSocketClient & operator=(const UnixSocketClient &) = delete;

  bool connect_once(std::string * error_message = nullptr);
  bool ensure_connected(std::string * error_message = nullptr);
  void close();
  bool connected() const;
  bool send_message(const std::string & message, std::string * error_message = nullptr);
  bool read_message(std::string * message, int timeout_ms, std::string * error_message = nullptr);

private:
  std::string socket_path_;
  int timeout_ms_{200};
  int fd_{-1};
  std::string read_buffer_;
  mutable std::mutex mutex_;
};

}  // namespace a2_unitree_ipc

#endif  // A2_UNITREE_IPC_CLIENT_HPP_
