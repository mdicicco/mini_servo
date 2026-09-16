#ifndef MINI_SERVO_HARDWARE__MINI_SERVO_HARDWARE_HPP_
#define MINI_SERVO_HARDWARE__MINI_SERVO_HARDWARE_HPP_

#include <chrono>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "mini_servo_hardware/serial_port.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace mini_servo_hardware
{

class MiniServoHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MiniServoHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::string param_or(const std::string & key, const std::string & fallback) const;
  void apply_startup_pose();
  bool send_position_command();
  void consume_serial_lines();
  bool parse_angle_line(const std::string & line, std::vector<double> & degrees) const;

  SerialPort serial_;
  std::string serial_port_{"/dev/ttyUSB0"};
  int baud_rate_{500000};
  std::chrono::milliseconds command_period_{50};

  std::vector<std::string> joint_names_;
  std::vector<double> hw_commands_rad_;
  std::vector<double> hw_positions_rad_;
  std::vector<double> last_sent_rad_;
  std::string rx_buffer_;
  std::chrono::steady_clock::time_point last_write_time_{};
  bool force_write_{true};
  bool awaiting_echo_{false};
};

}  // namespace mini_servo_hardware

#endif  // MINI_SERVO_HARDWARE__MINI_SERVO_HARDWARE_HPP_
