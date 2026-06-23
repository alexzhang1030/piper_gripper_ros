#include "piper_gripper_hardware/piper_gripper_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "piper_gripper_hardware/piper_gripper_protocol.hpp"

namespace
{
auto logger() -> rclcpp::Logger
{
  static auto const log = rclcpp::get_logger("PiperGripperHardware");
  return log;
}

auto now_ms(rclcpp::Clock& clock) -> std::uint64_t
{
  return static_cast<std::uint64_t>(clock.now().nanoseconds() / 1'000'000);
}

auto parse_bool_parameter(std::string_view value, bool fallback) -> bool
{
  if (value == "true" || value == "True" || value == "TRUE" || value == "1") {
    return true;
  }
  if (value == "false" || value == "False" || value == "FALSE" || value == "0") {
    return false;
  }
  return fallback;
}

auto frame_to_debug_payload(const can_frame& frame) -> std::vector<std::uint8_t>
{
  std::vector<std::uint8_t> payload;
  payload.reserve(13);
  const auto can_id = frame.can_id & CAN_EFF_MASK;
  payload.push_back(static_cast<std::uint8_t>(can_id & 0xFFU));
  payload.push_back(static_cast<std::uint8_t>((can_id >> 8U) & 0xFFU));
  payload.push_back(static_cast<std::uint8_t>((can_id >> 16U) & 0xFFU));
  payload.push_back(static_cast<std::uint8_t>((can_id >> 24U) & 0xFFU));
  payload.push_back(frame.can_dlc);
  for (std::size_t index = 0; index < 8; ++index) {
    payload.push_back(frame.data[index]);
  }
  return payload;
}
}  // namespace

namespace piper_gripper_hardware
{

#ifdef PIPER_GRIPPER_HAS_HARDWARE_COMPONENT_INTERFACE_PARAMS
auto PiperGripperHardware::on_init(const hardware_interface::HardwareComponentInterfaceParams& params)
    -> hardware_interface::CallbackReturn
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
#else
auto PiperGripperHardware::on_init(const hardware_interface::HardwareInfo& info) -> hardware_interface::CallbackReturn
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
#endif
    RCLCPP_ERROR(logger(), "Base on_init failed.");
    return CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 1) {
    RCLCPP_ERROR(logger(), "Expected exactly one gripper joint, got %zu", info_.joints.size());
    return CallbackReturn::ERROR;
  }

  can_interface_ = get_parameter_or("can_interface", can_interface_);
  command_can_id_ = static_cast<canid_t>(get_parameter_or("command_can_id", static_cast<int>(command_can_id_)));
  feedback_can_id_ = static_cast<canid_t>(get_parameter_or("feedback_can_id", static_cast<int>(feedback_can_id_)));
  min_position_m_ = get_parameter_or("min_position_m", min_position_m_);
  max_position_m_ = get_parameter_or("max_position_m", max_position_m_);
  initial_position_m_ = get_parameter_or("initial_position_m", initial_position_m_);
  command_deadband_m_ = std::max(0.0, get_parameter_or("command_deadband_m", command_deadband_m_));
  default_effort_mn_ = std::clamp(get_parameter_or("default_effort_mn", default_effort_mn_), 0, 5000);
  receive_timeout_ms_ = std::max(1, get_parameter_or("receive_timeout_ms", receive_timeout_ms_));
  feedback_timeout_ms_ = std::max(1, get_parameter_or("feedback_timeout_ms", feedback_timeout_ms_));
  command_refresh_interval_ms_ = std::max(1, get_parameter_or("command_refresh_interval_ms", command_refresh_interval_ms_));
  activate_set_zero_ = get_parameter_or("activate_set_zero", activate_set_zero_);
  publish_debug_commands_ = get_parameter_or("publish_debug_commands", publish_debug_commands_);
  publish_sent_position_ = get_parameter_or("publish_sent_position", publish_sent_position_);
  publish_feedback_position_ = get_parameter_or("publish_feedback_position", publish_feedback_position_);
  debug_topic_prefix_ = get_parameter_or("debug_topic_prefix", debug_topic_prefix_);

  if (min_position_m_ > max_position_m_) {
    std::swap(min_position_m_, max_position_m_);
  }
  initial_position_m_ = std::clamp(initial_position_m_, min_position_m_, max_position_m_);
  hw_position_ = initial_position_m_;
  hw_command_position_ = initial_position_m_;
  hw_position_export_ = initial_position_m_;

  if (publish_debug_commands_) {
    command_debug_publisher_ = debug_node()->create_publisher<std_msgs::msg::UInt8MultiArray>(
        make_debug_topic_name(debug_topic_prefix_, "gripper_command_debug"), rclcpp::SystemDefaultsQoS{});
    stamped_command_debug_publisher_ = debug_node()->create_publisher<std_msgs_stamped::msg::StampedUInt8MultiArray>(
        make_debug_topic_name(debug_topic_prefix_, "gripper_command_debug_stamped"), rclcpp::SystemDefaultsQoS{});
  }
  if (publish_sent_position_) {
    sent_position_publisher_ = debug_node()->create_publisher<std_msgs::msg::Float64>(
        make_debug_topic_name(debug_topic_prefix_, "gripper_sent_position_debug"), rclcpp::SystemDefaultsQoS{});
    stamped_sent_position_publisher_ = debug_node()->create_publisher<std_msgs_stamped::msg::StampedFloat64>(
        make_debug_topic_name(debug_topic_prefix_, "gripper_sent_position_debug_stamped"), rclcpp::SystemDefaultsQoS{});
  }
  if (publish_feedback_position_) {
    feedback_position_publisher_ = debug_node()->create_publisher<std_msgs::msg::UInt8MultiArray>(
        make_debug_topic_name(debug_topic_prefix_, "gripper_feedback_debug"), rclcpp::SystemDefaultsQoS{});
    stamped_feedback_position_publisher_ =
        debug_node()->create_publisher<std_msgs_stamped::msg::StampedUInt8MultiArray>(
            make_debug_topic_name(debug_topic_prefix_, "gripper_feedback_debug_stamped"), rclcpp::SystemDefaultsQoS{});
  }

  clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  RCLCPP_INFO(logger(),
              "on_init: interface=%s cmd_id=0x%X fb_id=0x%X range=[%.4f, %.4f] initial=%.4f effort=%d",
              can_interface_.c_str(), command_can_id_, feedback_can_id_, min_position_m_, max_position_m_,
              initial_position_m_, default_effort_mn_);
  return CallbackReturn::SUCCESS;
}

auto PiperGripperHardware::debug_node() -> rclcpp::Node::SharedPtr
{
#ifdef PIPER_GRIPPER_HAS_HARDWARE_COMPONENT_INTERFACE_PARAMS
  return get_node();
#else
  if (debug_node_ == nullptr) {
    debug_node_ = std::make_shared<rclcpp::Node>("piper_gripper_hardware_debug");
  }
  return debug_node_;
#endif
}

PiperGripperHardware::~PiperGripperHardware()
{
  comms_running_ = false;
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
}

auto PiperGripperHardware::on_configure(const rclcpp_lifecycle::State&) -> hardware_interface::CallbackReturn
{
  try {
    can_socket_ = std::make_unique<CanSocket>(can_interface_);
    if (!can_socket_->open(receive_timeout_ms_)) {
      RCLCPP_ERROR(logger(), "Failed to open CAN interface: %s", can_interface_.c_str());
      return CallbackReturn::ERROR;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger(), "Exception opening CAN interface: %s", e.what());
    return CallbackReturn::ERROR;
  }

  comms_running_ = true;
  first_feedback_received_ = false;
  receive_thread_ = std::thread(&PiperGripperHardware::receive_loop, this);
  RCLCPP_INFO(logger(), "CAN interface %s configured.", can_interface_.c_str());
  return CallbackReturn::SUCCESS;
}

auto PiperGripperHardware::on_activate(const rclcpp_lifecycle::State&) -> hardware_interface::CallbackReturn
{
  active_ = true;
  const auto activation_time = now_ms(*clock_);
  last_feedback_time_ms_ = activation_time;
  last_command_time_ms_ = 0;

  const auto current_position = hw_position_.load();
  hw_command_position_ = std::isfinite(current_position) ? current_position : initial_position_m_;

  const std::uint8_t set_zero = activate_set_zero_ == 0xAE ? 0xAE : 0x00;
  if (send_gripper_command(0x03, set_zero) == CanSendResult::kError) {
    active_ = false;
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger(), "Piper gripper hardware activated.");
  return CallbackReturn::SUCCESS;
}

auto PiperGripperHardware::on_deactivate(const rclcpp_lifecycle::State&) -> hardware_interface::CallbackReturn
{
  active_ = false;
  (void)send_gripper_command(0x00, 0x00);
  RCLCPP_INFO(logger(), "Piper gripper hardware deactivated.");
  return CallbackReturn::SUCCESS;
}

auto PiperGripperHardware::on_cleanup(const rclcpp_lifecycle::State&) -> hardware_interface::CallbackReturn
{
  active_ = false;
  comms_running_ = false;
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
  if (can_socket_ != nullptr) {
    can_socket_->close();
  }

  hw_command_position_ = std::numeric_limits<double>::quiet_NaN();
  hw_position_export_ = std::numeric_limits<double>::quiet_NaN();
  hw_effort_export_ = std::numeric_limits<double>::quiet_NaN();
  hw_status_code_export_ = 0.0;
  hw_enabled_export_ = 0.0;
  hw_homed_export_ = 0.0;
  hw_fault_export_ = 0.0;
  return CallbackReturn::SUCCESS;
}

auto PiperGripperHardware::on_shutdown(const rclcpp_lifecycle::State& previous_state)
    -> hardware_interface::CallbackReturn
{
  return on_cleanup(previous_state);
}

auto PiperGripperHardware::export_state_interfaces() -> std::vector<hardware_interface::StateInterface>
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_position_export_);
  state_interfaces.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_EFFORT, &hw_effort_export_);
  state_interfaces.emplace_back(info_.joints[0].name, "status_code", &hw_status_code_export_);
  state_interfaces.emplace_back(info_.joints[0].name, "enabled", &hw_enabled_export_);
  state_interfaces.emplace_back(info_.joints[0].name, "homed", &hw_homed_export_);
  state_interfaces.emplace_back(info_.joints[0].name, "fault", &hw_fault_export_);
  return state_interfaces;
}

auto PiperGripperHardware::export_command_interfaces() -> std::vector<hardware_interface::CommandInterface>
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_command_position_);
  return command_interfaces;
}

auto PiperGripperHardware::read(const rclcpp::Time&, const rclcpp::Duration&) -> hardware_interface::return_type
{
  if (can_socket_ == nullptr || !can_socket_->is_open()) {
    return hardware_interface::return_type::ERROR;
  }

  const auto now = now_ms(*clock_);
  const auto last_feedback = last_feedback_time_ms_.load();
  if (active_ && (now - last_feedback > static_cast<std::uint64_t>(feedback_timeout_ms_))) {
    RCLCPP_ERROR_THROTTLE(logger(), *clock_, 1000,
                          "Piper gripper feedback timeout: no feedback for %d ms (first_feedback_received=%s).",
                          feedback_timeout_ms_, first_feedback_received_.load() ? "true" : "false");
    return hardware_interface::return_type::ERROR;
  }

  hw_position_export_ = hw_position_.load();
  hw_effort_export_ = hw_effort_.load();
  hw_status_code_export_ = static_cast<double>(hw_status_code_.load());
  hw_enabled_export_ = hw_enabled_.load() ? 1.0 : 0.0;
  hw_homed_export_ = hw_homed_.load() ? 1.0 : 0.0;
  hw_fault_export_ = hw_fault_.load() ? 1.0 : 0.0;
  return hardware_interface::return_type::OK;
}

auto PiperGripperHardware::write(const rclcpp::Time&, const rclcpp::Duration&) -> hardware_interface::return_type
{
  if (!active_) {
    return hardware_interface::return_type::OK;
  }
  if (can_socket_ == nullptr || !can_socket_->is_open()) {
    RCLCPP_ERROR(logger(), "CAN interface is not open.");
    return hardware_interface::return_type::ERROR;
  }

  const double command = hw_command_position_;
  if (!std::isfinite(command)) {
    return hardware_interface::return_type::OK;
  }

  const auto clamped = std::clamp(command, min_position_m_, max_position_m_);
  const auto previous = hw_position_.load();
  const bool changed = !std::isfinite(previous) || std::fabs(clamped - previous) >= command_deadband_m_;
  const auto now = now_ms(*clock_);
  const bool refresh_due =
      (now - last_command_time_ms_.load()) >= static_cast<std::uint64_t>(command_refresh_interval_ms_);

  if (!changed && !refresh_due) {
    return hardware_interface::return_type::OK;
  }

  if (send_gripper_command(0x01, 0x00) == CanSendResult::kError) {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

void PiperGripperHardware::receive_loop()
{
  while (comms_running_) {
    if (can_socket_ == nullptr || !can_socket_->is_open()) {
      rclcpp::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    can_frame frame{};
    if (!can_socket_->receive(frame)) {
      continue;
    }

    const auto feedback = protocol::parse_feedback_frame(frame, feedback_can_id_);
    if (!feedback.has_value()) {
      continue;
    }
    publish_feedback_position_frame(frame);

    hw_position_ = std::clamp(feedback->position_m, min_position_m_, max_position_m_);
    hw_effort_ = feedback->effort_nm;
    hw_status_code_ = feedback->status_code;
    hw_enabled_ = feedback->driver_enabled;
    hw_homed_ = feedback->homed;
    hw_fault_ = feedback->voltage_too_low || feedback->motor_overheating || feedback->driver_overcurrent ||
                feedback->driver_overheating || feedback->sensor_fault || feedback->driver_fault;
    first_feedback_received_ = true;
    last_feedback_time_ms_ = now_ms(*clock_);
  }
}

auto PiperGripperHardware::send_gripper_command(std::uint8_t status_code, std::uint8_t set_zero) -> CanSendResult
{
  std::scoped_lock lock(can_send_mutex_);
  if (can_socket_ == nullptr || !can_socket_->is_open()) {
    return CanSendResult::kError;
  }

  const auto clamped = std::clamp(hw_command_position_, min_position_m_, max_position_m_);
  const auto frame = protocol::make_command_frame(
      command_can_id_, protocol::Command{
                           .position_m = clamped,
                           .effort_mn = static_cast<std::uint16_t>(default_effort_mn_),
                           .status_code = status_code,
                           .set_zero = set_zero,
                       });
  const auto result = can_socket_->send(frame);
  if (result == CanSendResult::kError) {
    RCLCPP_ERROR(logger(), "Failed to send gripper command frame.");
    return result;
  }
  if (result == CanSendResult::kTxQueueFull) {
    RCLCPP_WARN_THROTTLE(logger(), *clock_, 1000, "CAN TX queue full, will retry next control cycle.");
    return result;
  }
  publish_command_debug_frame(frame);
  publish_sent_position_value(clamped);
  last_command_time_ms_ = now_ms(*clock_);
  return result;
}

auto PiperGripperHardware::get_parameter_or(const std::string& name, const std::string& fallback) const -> std::string
{
  const auto iter = info_.hardware_parameters.find(name);
  return iter == info_.hardware_parameters.end() ? fallback : iter->second;
}

auto PiperGripperHardware::get_parameter_or(const std::string& name, int fallback) const -> int
{
  const auto iter = info_.hardware_parameters.find(name);
  return iter == info_.hardware_parameters.end() ? fallback : std::stoi(iter->second);
}

auto PiperGripperHardware::get_parameter_or(const std::string& name, double fallback) const -> double
{
  const auto iter = info_.hardware_parameters.find(name);
  return iter == info_.hardware_parameters.end() ? fallback : std::stod(iter->second);
}

auto PiperGripperHardware::get_parameter_or(const std::string& name, bool fallback) const -> bool
{
  const auto iter = info_.hardware_parameters.find(name);
  return iter == info_.hardware_parameters.end() ? fallback : parse_bool_parameter(iter->second, fallback);
}

auto PiperGripperHardware::make_debug_topic_name(const std::string& prefix, const std::string& base_name) -> std::string
{
  if (prefix.empty()) {
    return "~/" + base_name;
  }
  std::string_view p = prefix;
  if (p.front() == '/') {
    p.remove_prefix(1);
  }

  std::string result = "~/";
  result += p;
  if (result.back() != '/') {
    result += '/';
  }
  result += base_name;
  return result;
}

void PiperGripperHardware::publish_command_debug_frame(const can_frame& frame)
{
  if (!publish_debug_commands_) {
    return;
  }
  const auto payload = frame_to_debug_payload(frame);
  const auto stamp = clock_->now();
  if (command_debug_publisher_ != nullptr) {
    std_msgs::msg::UInt8MultiArray msg{};
    msg.data = payload;
    command_debug_publisher_->publish(msg);
  }
  if (stamped_command_debug_publisher_ != nullptr) {
    std_msgs_stamped::msg::StampedUInt8MultiArray msg{};
    msg.header.stamp = stamp;
    msg.header.frame_id.clear();
    msg.layout = std_msgs::msg::MultiArrayLayout{};
    msg.data = payload;
    stamped_command_debug_publisher_->publish(msg);
  }
}

void PiperGripperHardware::publish_sent_position_value(double position)
{
  if (!publish_sent_position_) {
    return;
  }
  if (sent_position_publisher_ != nullptr) {
    std_msgs::msg::Float64 msg{};
    msg.data = position;
    sent_position_publisher_->publish(msg);
  }
  if (stamped_sent_position_publisher_ != nullptr) {
    std_msgs_stamped::msg::StampedFloat64 msg{};
    msg.header.stamp = clock_->now();
    msg.header.frame_id.clear();
    msg.data = position;
    stamped_sent_position_publisher_->publish(msg);
  }
}

void PiperGripperHardware::publish_feedback_position_frame(const can_frame& frame)
{
  if (!publish_feedback_position_) {
    return;
  }
  const auto payload = frame_to_debug_payload(frame);
  const auto stamp = clock_->now();
  if (feedback_position_publisher_ != nullptr) {
    std_msgs::msg::UInt8MultiArray msg{};
    msg.data = payload;
    feedback_position_publisher_->publish(msg);
  }
  if (stamped_feedback_position_publisher_ != nullptr) {
    std_msgs_stamped::msg::StampedUInt8MultiArray msg{};
    msg.header.stamp = stamp;
    msg.header.frame_id.clear();
    msg.layout = std_msgs::msg::MultiArrayLayout{};
    msg.data = payload;
    stamped_feedback_position_publisher_->publish(msg);
  }
}

}  // namespace piper_gripper_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(piper_gripper_hardware::PiperGripperHardware, hardware_interface::SystemInterface)
