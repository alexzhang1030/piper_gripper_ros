#ifndef PIPER_GRIPPER_HARDWARE_CAN_SOCKET_HPP_
#define PIPER_GRIPPER_HARDWARE_CAN_SOCKET_HPP_

#include <linux/can.h>

#include <cstdint>
#include <string>

namespace piper_gripper_hardware
{

enum class CanSendResult : std::uint8_t
{
  kOk,
  kTxQueueFull,
  kError,
};

class CanSocket
{
public:
  explicit CanSocket(std::string interface_name);
  CanSocket(const CanSocket&) = delete;
  CanSocket(CanSocket&&) = delete;
  auto operator=(const CanSocket&) -> CanSocket& = delete;
  auto operator=(CanSocket&&) -> CanSocket& = delete;
  ~CanSocket();

  auto open(int receive_timeout_ms) -> bool;
  void close();
  [[nodiscard]] auto is_open() const -> bool;

  [[nodiscard]] auto send(const can_frame& frame) const -> CanSendResult;
  auto receive(can_frame& frame) const -> bool;

private:
  int socket_fd_{-1};
  std::string interface_name_;
};

}  // namespace piper_gripper_hardware

#endif  // PIPER_GRIPPER_HARDWARE_CAN_SOCKET_HPP_
