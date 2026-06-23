#ifndef PIPER_GRIPPER_HARDWARE_PIPER_GRIPPER_PROTOCOL_HPP_
#define PIPER_GRIPPER_HARDWARE_PIPER_GRIPPER_PROTOCOL_HPP_

#include <linux/can.h>

#include <cstdint>
#include <optional>

namespace piper_gripper_hardware::protocol
{

constexpr canid_t kGripperCommandCanId = 0x159;
constexpr canid_t kGripperFeedbackCanId = 0x2A8;

struct Command
{
  double position_m{0.0};
  std::uint16_t effort_mn{1000};  // 0.001 N*m per unit
  std::uint8_t status_code{0x01};
  std::uint8_t set_zero{0x00};
};

struct Feedback
{
  double position_m{0.0};
  double effort_nm{0.0};
  std::uint8_t status_code{0x00};
  bool voltage_too_low{false};
  bool motor_overheating{false};
  bool driver_overcurrent{false};
  bool driver_overheating{false};
  bool sensor_fault{false};
  bool driver_fault{false};
  bool driver_enabled{false};
  bool homed{false};
};

auto make_command_frame(canid_t can_id, const Command& command) -> can_frame;
auto parse_feedback_frame(const can_frame& frame, canid_t expected_can_id) -> std::optional<Feedback>;

}  // namespace piper_gripper_hardware::protocol

#endif  // PIPER_GRIPPER_HARDWARE_PIPER_GRIPPER_PROTOCOL_HPP_
