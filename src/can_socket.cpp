#include "piper_gripper_hardware/can_socket.hpp"

#include <linux/can/raw.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <net/if.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <utility>

namespace piper_gripper_hardware
{

CanSocket::CanSocket(std::string interface_name) : interface_name_(std::move(interface_name))
{
}

CanSocket::~CanSocket()
{
  close();
}

auto CanSocket::open(int receive_timeout_ms) -> bool
{
  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    perror("Error opening CAN socket");
    return false;
  }

  timeval receive_timeout{};
  receive_timeout.tv_sec = receive_timeout_ms / 1000;
  receive_timeout.tv_usec = (receive_timeout_ms % 1000) * 1000;
  if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &receive_timeout, sizeof(receive_timeout)) != 0) {
    perror("Error setting CAN receive timeout");
    close();
    return false;
  }

  ifreq ifr{};
  strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    perror("Error getting CAN interface index");
    close();
    return false;
  }

  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(socket_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    perror("Error binding CAN socket");
    close();
    return false;
  }

  return true;
}

void CanSocket::close()
{
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

auto CanSocket::is_open() const -> bool
{
  return socket_fd_ >= 0;
}

auto CanSocket::send(const can_frame& frame) const -> CanSendResult
{
  if (!is_open()) {
    return CanSendResult::kError;
  }
  const auto written = write(socket_fd_, &frame, sizeof(can_frame));
  if (written == static_cast<ssize_t>(sizeof(can_frame))) {
    return CanSendResult::kOk;
  }
  if (written < 0 && (errno == ENOBUFS || errno == EAGAIN || errno == EWOULDBLOCK)) {
    return CanSendResult::kTxQueueFull;
  }
  perror("Error writing CAN frame");
  return CanSendResult::kError;
}

auto CanSocket::receive(can_frame& frame) const -> bool
{
  if (!is_open()) {
    return false;
  }
  const auto read_bytes = read(socket_fd_, &frame, sizeof(can_frame));
  if (read_bytes != static_cast<ssize_t>(sizeof(can_frame))) {
    if (errno != EAGAIN && errno != EWOULDBLOCK) {
      perror("Error reading CAN frame");
    }
    return false;
  }
  return true;
}

}  // namespace piper_gripper_hardware
