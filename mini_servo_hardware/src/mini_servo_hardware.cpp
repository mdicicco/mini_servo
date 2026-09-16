#include "mini_servo_hardware/mini_servo_hardware.hpp"

#include <cmath>
#include <cstdlib>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace mini_servo_hardware
{
namespace
{
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
constexpr size_t kDof = 6;
// Matches the Arduino firmware startup pose (degrees).
constexpr double kStartupDeg[kDof] = {0.0, 0.0, -70.0, -15.0, 45.0, -5.0};
}  // namespace

std::string MiniServoHardware::param_or(
  const std::string & key, const std::string & fallback) const
{
  const auto it = info_.hardware_parameters.find(key);
  if (it == info_.hardware_parameters.end()) {
    return fallback;
  }
  return it->second;
}

hardware_interface::CallbackReturn MiniServoHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  serial_port_ = param_or("serial_port", "/dev/ttyUSB0");
  baud_rate_ = std::stoi(param_or("baud_rate", "500000"));
  command_period_ =
    std::chrono::milliseconds(std::stoi(param_or("command_period_ms", "500")));

  if (info_.joints.size() != kDof) {
    RCLCPP_FATAL(
      get_logger(), "Expected %zu joints, got %zu", kDof, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.clear();
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' must have a single position command interface",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    joint_names_.push_back(joint.name);
  }

  hw_commands_rad_.assign(kDof, 0.0);
  hw_positions_rad_.assign(kDof, 0.0);
  last_sent_rad_.assign(kDof, 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

void MiniServoHardware::apply_startup_pose()
{
  for (size_t i = 0; i < kDof; ++i) {
    const double rad = kStartupDeg[i] * kDegToRad;
    hw_commands_rad_[i] = rad;
    hw_positions_rad_[i] = rad;
    last_sent_rad_[i] = rad;
    set_state(joint_names_[i] + "/" + hardware_interface::HW_IF_POSITION, rad);
    set_state(joint_names_[i] + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
    set_command(joint_names_[i] + "/" + hardware_interface::HW_IF_POSITION, rad);
  }
}

hardware_interface::CallbackReturn MiniServoHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    get_logger(), "Opening serial port %s at %d baud", serial_port_.c_str(), baud_rate_);
  if (!serial_.open(serial_port_, baud_rate_)) {
    RCLCPP_FATAL(
      get_logger(), "Failed to open serial port %s", serial_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  apply_startup_pose();
  rx_buffer_.clear();
  force_write_ = true;
  awaiting_echo_ = false;
  last_write_time_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(get_logger(), "Serial port configured");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MiniServoHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  apply_startup_pose();
  force_write_ = true;
  RCLCPP_INFO(get_logger(), "Mini servo hardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MiniServoHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Mini servo hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MiniServoHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  serial_.close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool MiniServoHardware::parse_angle_line(
  const std::string & line, std::vector<double> & degrees) const
{
  std::vector<double> values;
  std::stringstream ss(line);
  std::string token;
  while (std::getline(ss, token, ',')) {
    if (token.empty()) {
      continue;
    }
    try {
      values.push_back(std::stod(token));
    } catch (const std::exception &) {
      return false;
    }
  }
  if (values.size() != kDof) {
    return false;
  }
  // The firmware also prints PWM ticks periodically (all values > 90).
  bool likely_pwm = true;
  for (const double value : values) {
    if (value < 90.0) {
      likely_pwm = false;
      break;
    }
  }
  if (likely_pwm) {
    return false;
  }
  degrees = values;
  return true;
}

void MiniServoHardware::consume_serial_lines()
{
  rx_buffer_ += serial_.read_available();
  size_t newline = rx_buffer_.find('\n');
  while (newline != std::string::npos) {
    std::string line = rx_buffer_.substr(0, newline);
    rx_buffer_.erase(0, newline + 1);
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    std::vector<double> degrees;
    if (parse_angle_line(line, degrees)) {
      for (size_t i = 0; i < kDof; ++i) {
        hw_positions_rad_[i] = degrees[i] * kDegToRad;
      }
      awaiting_echo_ = false;
    }
    newline = rx_buffer_.find('\n');
  }
}

bool MiniServoHardware::send_position_command()
{
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(4);
  for (size_t i = 0; i < kDof; ++i) {
    if (i > 0) {
      oss << ", ";
    }
    oss << (hw_commands_rad_[i] * kRadToDeg);
  }
  oss << '\n';
  return serial_.write(oss.str());
}

hardware_interface::return_type MiniServoHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  consume_serial_lines();
  for (size_t i = 0; i < kDof; ++i) {
    set_state(
      joint_names_[i] + "/" + hardware_interface::HW_IF_POSITION, hw_positions_rad_[i]);
    set_state(joint_names_[i] + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MiniServoHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  bool changed = force_write_;
  for (size_t i = 0; i < kDof; ++i) {
    hw_commands_rad_[i] =
      get_command(joint_names_[i] + "/" + hardware_interface::HW_IF_POSITION);
    if (!std::isfinite(hw_commands_rad_[i])) {
      RCLCPP_ERROR(get_logger(), "Non-finite command for %s", joint_names_[i].c_str());
      return hardware_interface::return_type::ERROR;
    }
    if (std::abs(hw_commands_rad_[i] - last_sent_rad_[i]) > 1e-4) {
      changed = true;
    }
  }

  const auto now = std::chrono::steady_clock::now();
  if (awaiting_echo_ && (now - last_write_time_) < std::chrono::seconds(1)) {
    return hardware_interface::return_type::OK;
  }
  if (!changed && (now - last_write_time_) < command_period_) {
    return hardware_interface::return_type::OK;
  }

  // The Arduino sketch reads one byte per 10 ms loop, so keep writes slow.
  if (!send_position_command()) {
    RCLCPP_ERROR(get_logger(), "Failed to write serial command");
    return hardware_interface::return_type::ERROR;
  }
  last_sent_rad_ = hw_commands_rad_;
  last_write_time_ = now;
  force_write_ = false;
  awaiting_echo_ = true;
  return hardware_interface::return_type::OK;
}

}  // namespace mini_servo_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  mini_servo_hardware::MiniServoHardware, hardware_interface::SystemInterface)
