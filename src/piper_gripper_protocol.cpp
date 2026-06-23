#include "piper_gripper_hardware/piper_gripper_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

namespace piper_gripper_hardware::protocol
{
namespace
{
auto as_i32_be(const std::uint8_t* bytes) -> std::int32_t
{
  std::uint32_t value = 0;
  value |= static_cast<std::uint32_t>(bytes[0]) << 24;
  value |= static_cast<std::uint32_t>(bytes[1]) << 16;
  value |= static_cast<std::uint32_t>(bytes[2]) << 8;
  value |= static_cast<std::uint32_t>(bytes[3]);
  return static_cast<std::int32_t>(value);
}

auto as_u16_be(const std::uint8_t* bytes) -> std::uint16_t
{
  std::uint16_t value = 0;
  value |= static_cast<std::uint16_t>(bytes[0]) << 8;
  value |= static_cast<std::uint16_t>(bytes[1]);
  return value;
}

void set_i32_be(std::uint8_t* bytes, std::int32_t value)
{
  const auto as_u32 = static_cast<std::uint32_t>(value);
  bytes[0] = static_cast<std::uint8_t>((as_u32 >> 24) & 0xFF);
  bytes[1] = static_cast<std::uint8_t>((as_u32 >> 16) & 0xFF);
  bytes[2] = static_cast<std::uint8_t>((as_u32 >> 8) & 0xFF);
  bytes[3] = static_cast<std::uint8_t>(as_u32 & 0xFF);
}

void set_u16_be(std::uint8_t* bytes, std::uint16_t value)
{
  bytes[0] = static_cast<std::uint8_t>((value >> 8) & 0xFF);
  bytes[1] = static_cast<std::uint8_t>(value & 0xFF);
}
}  // namespace

auto make_command_frame(canid_t can_id, const Command& command) -> can_frame
{
  can_frame frame{};
  frame.can_id = can_id;
  frame.can_dlc = 8;

  const double clamped_position_m = std::clamp(command.position_m, -2.147483647, 2.147483647);
  const auto gripper_angle = static_cast<std::int32_t>(std::lround(clamped_position_m * 1'000'000.0));
  const auto effort = static_cast<std::uint16_t>(std::clamp(static_cast<int>(command.effort_mn), 0, 5000));

  set_i32_be(&frame.data[0], gripper_angle);
  set_u16_be(&frame.data[4], effort);
  frame.data[6] = command.status_code;
  frame.data[7] = command.set_zero;
  return frame;
}

auto parse_feedback_frame(const can_frame& frame, canid_t expected_can_id) -> std::optional<Feedback>
{
  if ((frame.can_id & CAN_EFF_MASK) != expected_can_id || frame.can_dlc < 7) {
    return std::nullopt;
  }

  Feedback feedback{};
  const auto gripper_angle = as_i32_be(&frame.data[0]);
  const auto gripper_effort = static_cast<std::int16_t>(as_u16_be(&frame.data[4]));
  feedback.position_m = static_cast<double>(gripper_angle) / 1'000'000.0;
  feedback.effort_nm = static_cast<double>(gripper_effort) / 1'000.0;
  feedback.status_code = frame.data[6];
  feedback.voltage_too_low = (feedback.status_code & (1U << 0U)) != 0U;
  feedback.motor_overheating = (feedback.status_code & (1U << 1U)) != 0U;
  feedback.driver_overcurrent = (feedback.status_code & (1U << 2U)) != 0U;
  feedback.driver_overheating = (feedback.status_code & (1U << 3U)) != 0U;
  feedback.sensor_fault = (feedback.status_code & (1U << 4U)) != 0U;
  feedback.driver_fault = (feedback.status_code & (1U << 5U)) != 0U;
  feedback.driver_enabled = (feedback.status_code & (1U << 6U)) != 0U;
  feedback.homed = (feedback.status_code & (1U << 7U)) != 0U;
  return feedback;
}

}  // namespace piper_gripper_hardware::protocol
