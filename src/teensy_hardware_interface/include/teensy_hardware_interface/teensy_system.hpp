#ifndef TEENSY_HARDWARE_INTERFACE__TEENSY_SYSTEM_HPP_
#define TEENSY_HARDWARE_INTERFACE__TEENSY_SYSTEM_HPP_

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/hardware_info.hpp>

#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include <string>
#include <vector>

namespace teensy_hardware_interface
{

class TeensySystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  hardware_interface::HardwareInfo info_;

  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> commands_;

  // Per-joint mapping tweaks (ROS<->HW)
  std::vector<double> sign_;    // +1 or -1
  std::vector<double> scale_;   // 1.0 for radians; 57.2957795 if Teensy expects degrees
  std::vector<double> offset_;  // optional constant offset in ROS units (radians)

  // Indices by joint name (robust even if order changes)
  int idx_base_{-1};   // "continuous"
  int idx_arm1_{-1};   // "revolute1"
  int idx_arm2_{-1};   // "revolute2"
  int idx_wrist_{-1};  // "revolute3"

  // Serial connection to Teensy
  std::string port_;
  int baudrate_{115200};
  int fd_{-1};

  // Optional behavior
  bool send_auto_on_activate_{false};

  // Logging throttle
  int log_div_{20};
};

}  // namespace teensy_hardware_interface

#endif  // TEENSY_HARDWARE_INTERFACE__TEENSY_SYSTEM_HPP_

