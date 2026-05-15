#pragma once

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>

#include <algorithm>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace a2_system
{

struct InterfaceInfo
{
  std::string name;
  std::vector<std::string> ipv4_addrs;
  bool up{false};
  bool lower_up{false};
  bool loopback{false};
};

inline bool is_virtual_like_name(const std::string & name)
{
  static const std::vector<std::string> prefixes = {
    "docker", "br-", "veth", "virbr", "vmnet", "wl", "tun", "tap", "tailscale", "Meta"};
  for (const auto & prefix : prefixes) {
    if (name.rfind(prefix, 0) == 0) {
      return true;
    }
  }
  return false;
}

inline int interface_priority(const std::string & name)
{
  if (name.rfind("enx", 0) == 0) {
    return 0;
  }
  if (name.rfind("en", 0) == 0 || name.rfind("eth", 0) == 0) {
    return 1;
  }
  if (name == "lo") {
    return 9;
  }
  return 5;
}

inline std::vector<InterfaceInfo> discover_interfaces()
{
  std::map<std::string, InterfaceInfo> by_name;
  ifaddrs * addrs = nullptr;
  if (getifaddrs(&addrs) != 0 || addrs == nullptr) {
    return {};
  }

  for (ifaddrs * cursor = addrs; cursor != nullptr; cursor = cursor->ifa_next) {
    if (cursor->ifa_name == nullptr) {
      continue;
    }

    auto & info = by_name[cursor->ifa_name];
    info.name = cursor->ifa_name;
    info.up = info.up || ((cursor->ifa_flags & IFF_UP) != 0);
    info.lower_up = info.lower_up || ((cursor->ifa_flags & IFF_RUNNING) != 0);
    info.loopback = info.loopback || ((cursor->ifa_flags & IFF_LOOPBACK) != 0);

    if (cursor->ifa_addr == nullptr || cursor->ifa_addr->sa_family != AF_INET) {
      continue;
    }

    char buffer[INET_ADDRSTRLEN] = {0};
    auto * sin = reinterpret_cast<sockaddr_in *>(cursor->ifa_addr);
    if (inet_ntop(AF_INET, &sin->sin_addr, buffer, sizeof(buffer)) != nullptr) {
      info.ipv4_addrs.emplace_back(buffer);
    }
  }

  freeifaddrs(addrs);

  std::vector<InterfaceInfo> result;
  result.reserve(by_name.size());
  for (auto & item : by_name) {
    auto & info = item.second;
    std::sort(info.ipv4_addrs.begin(), info.ipv4_addrs.end());
    info.ipv4_addrs.erase(
      std::unique(info.ipv4_addrs.begin(), info.ipv4_addrs.end()),
      info.ipv4_addrs.end());
    result.push_back(info);
  }

  std::sort(
    result.begin(), result.end(),
    [](const InterfaceInfo & lhs, const InterfaceInfo & rhs) {
      if (interface_priority(lhs.name) != interface_priority(rhs.name)) {
        return interface_priority(lhs.name) < interface_priority(rhs.name);
      }
      return lhs.name < rhs.name;
    });
  return result;
}

inline bool interface_exists(const std::string & name)
{
  if (name.empty()) {
    return false;
  }

  const auto interfaces = discover_interfaces();
  return std::any_of(
    interfaces.begin(), interfaces.end(),
    [&](const InterfaceInfo & info) { return info.name == name; });
}

inline std::optional<InterfaceInfo> get_interface_info(const std::string & name)
{
  const auto interfaces = discover_interfaces();
  for (const auto & info : interfaces) {
    if (info.name == name) {
      return info;
    }
  }
  return std::nullopt;
}

inline bool interface_has_ipv4(const std::string & name)
{
  const auto info = get_interface_info(name);
  return info.has_value() && !info->ipv4_addrs.empty();
}

inline bool interface_is_ready_for_real(const std::string & name)
{
  const auto info = get_interface_info(name);
  if (!info.has_value()) {
    return false;
  }
  if (info->loopback || is_virtual_like_name(info->name)) {
    return false;
  }
  if (!info->up) {
    return false;
  }
  return info->lower_up || !info->ipv4_addrs.empty();
}

inline std::vector<std::string> candidate_interface_names(bool allow_loopback = false)
{
  std::vector<std::string> result;
  for (const auto & info : discover_interfaces()) {
    if (!info.up) {
      continue;
    }
    if (!allow_loopback && info.loopback) {
      continue;
    }
    if (is_virtual_like_name(info.name)) {
      continue;
    }
    result.push_back(info.name);
  }
  return result;
}

inline std::string select_interface(
  const std::string & preferred,
  const std::vector<std::string> & configured_candidates,
  bool allow_loopback = false)
{
  if (interface_exists(preferred)) {
    return preferred;
  }

  for (const auto & candidate : configured_candidates) {
    if (interface_exists(candidate)) {
      return candidate;
    }
  }

  const auto discovered = candidate_interface_names(allow_loopback);
  if (!discovered.empty()) {
    return discovered.front();
  }

  if (allow_loopback && interface_exists("lo")) {
    return "lo";
  }
  return {};
}

inline std::string describe_interfaces()
{
  std::ostringstream stream;
  bool first = true;
  for (const auto & info : discover_interfaces()) {
    if (!first) {
      stream << "; ";
    }
    first = false;
    stream << info.name << "[up=" << (info.up ? "true" : "false")
           << ", loopback=" << (info.loopback ? "true" : "false")
           << ", ipv4=";
    if (info.ipv4_addrs.empty()) {
      stream << "-";
    } else {
      for (std::size_t index = 0; index < info.ipv4_addrs.size(); ++index) {
        if (index != 0U) {
          stream << ",";
        }
        stream << info.ipv4_addrs[index];
      }
    }
    stream << "]";
  }
  return stream.str();
}

}  // namespace a2_system
