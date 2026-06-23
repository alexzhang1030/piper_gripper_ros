#ifndef PIPER_GRIPPER_HARDWARE_PIPER_GRIPPER_HARDWARE_HPP_
#define PIPER_GRIPPER_HARDWARE_PIPER_GRIPPER_HARDWARE_HPP_

#include <atomic>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#if __has_include("hardware_interface/types/hardware_component_interface_params.hpp")
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#define PIPER_GRIPPER_HAS_HARDWARE_COMPONENT_INTERFACE_PARAMS 1
#else
#include "hardware_interface/hardware_info.hpp"
#endif
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "piper_gripper_hardware/can_socket.hpp"
#include "piper_gripper_hardware/piper_gripper_protocol.hpp"

namespace piper_gripper_hardware
{

class PiperGripperHardware : public hardware_interface::SystemInterface
{
public:
  PiperGripperHardware() = default;
  PiperGripperHardware(const PiperGripperHardware&) = delete;
  PiperGripperHardware(PiperGripperHardware&&) = delete;
  auto operator=(const PiperGripperHardware&) -> PiperGripperHardware& = delete;
  auto operator=(PiperGripperHardware&&) -> PiperGripperHardware& = delete;
  ~PiperGripperHardware() override;

#ifdef PIPER_GRIPPER_HAS_HARDWARE_COMPONENT_INTERFACE_PARAMS
  auto on_init(const hardware_interface::HardwareComponentInterfaceParams& params)
      -> hardware_interface::CallbackReturn override;
#else
  auto on_init(const hardware_interface::HardwareInfo& info) -> hardware_interface::CallbackReturn override;
#endif
  auto on_configure(const rclcpp_lifecycle::State& previous_state) -> hardware_interface::CallbackReturn override;
  auto on_activate(const rclcpp_lifecycle::State& previous_state) -> hardware_interface::CallbackReturn override;
  auto on_deactivate(const rclcpp_lifecycle::State& previous_state) -> hardware_interface::CallbackReturn override;
  auto on_cleanup(const rclcpp_lifecycle::State& previous_state) -> hardware_interface::CallbackReturn override;
  auto on_shutdown(const rclcpp_lifecycle::State& previous_state) -> hardware_interface::CallbackReturn override;

  auto export_state_interfaces() -> std::vector<hardware_interface::StateInterface> override;
  auto export_command_interfaces() -> std::vector<hardware_interface::CommandInterface> override;

  auto read(const rclcpp::Time& time, const rclcpp::Duration& period) -> hardware_interface::return_type override;
  auto write(const rclcpp::Time& time, const rclcpp::Duration& period) -> hardware_interface::return_type override;

private:
  void receive_loop();
  auto send_gripper_command(std::uint8_t status_code, std::uint8_t set_zero) -> CanSendResult;
  auto get_parameter_or(const std::string& name, const std::string& fallback) const -> std::string;
  auto get_parameter_or(const std::string& name, int fallback) const -> int;
  auto get_parameter_or(const std::string& name, double fallback) const -> double;

  std::shared_ptr<rclcpp::Clock> clock_;
  std::unique_ptr<CanSocket> can_socket_;
  std::thread receive_thread_;
  std::atomic<bool> comms_running_{false};
  std::atomic<bool> active_{false};

  std::string can_interface_{"can0"};
  canid_t command_can_id_{protocol::kGripperCommandCanId};
  canid_t feedback_can_id_{protocol::kGripperFeedbackCanId};
  double min_position_m_{0.0};
  double max_position_m_{0.08};
  double initial_position_m_{0.0};
  double command_deadband_m_{1e-5};
  int default_effort_mn_{1000};
  int receive_timeout_ms_{50};
  int feedback_timeout_ms_{1000};
  int command_refresh_interval_ms_{100};
  int activate_set_zero_{0};

  std::mutex can_send_mutex_;
  std::atomic<std::uint64_t> last_feedback_time_ms_{0};
  std::atomic<std::uint64_t> last_command_time_ms_{0};
  std::atomic<bool> first_feedback_received_{false};

  std::atomic<double> hw_position_{std::numeric_limits<double>::quiet_NaN()};
  std::atomic<double> hw_effort_{std::numeric_limits<double>::quiet_NaN()};
  std::atomic<std::uint8_t> hw_status_code_{0};
  std::atomic<bool> hw_enabled_{false};
  std::atomic<bool> hw_homed_{false};
  std::atomic<bool> hw_fault_{false};

  double hw_command_position_{std::numeric_limits<double>::quiet_NaN()};
  double hw_position_export_{std::numeric_limits<double>::quiet_NaN()};
  double hw_effort_export_{std::numeric_limits<double>::quiet_NaN()};
  double hw_status_code_export_{0.0};
  double hw_enabled_export_{0.0};
  double hw_homed_export_{0.0};
  double hw_fault_export_{0.0};
};

}  // namespace piper_gripper_hardware

#endif  // PIPER_GRIPPER_HARDWARE_PIPER_GRIPPER_HARDWARE_HPP_
