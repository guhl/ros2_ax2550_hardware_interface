#ifndef AX2550_HARDWARE_INTERFACE__AX2550_HARDWARE_INTERFACE_HPP_
#define AX2550_HARDWARE_INTERFACE__AX2550_HARDWARE_INTERFACE_HPP_

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "ax2550_hardware_interface/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ax2550_driver/ax2550_comms.hpp"

namespace ax2550_hardware_interface
{

struct Config
{
  std::string port;
  std::string cmd_dir_fw;
  std::string cmd_dir_bw;
  std::string cmd_dir_le;
  std::string cmd_dir_ri;
  uint ticks_per_round;
  uint max_a;
  uint max_b;
  double speed_constant_x;
  double speed_constant_z;
  double wheel_diameter;
};

struct csv_reader: std::ctype<char> {
    csv_reader(): std::ctype<char>(get_table()) {}
    static std::ctype_base::mask const* get_table() {
        static std::vector<std::ctype_base::mask> rc(table_size, std::ctype_base::mask());

        rc[','] = std::ctype_base::space;
        rc['\n'] = std::ctype_base::space;
        rc[' '] = std::ctype_base::space;
        return &rc[0];
    }
}; 

template<typename T> std::vector<T> parse_parameter(const std::string & parameter){
  std::vector<T> result;
  std::stringstream ss(parameter);
  ss.imbue(std::locale(std::locale(), new csv_reader()));
  std::copy(std::istream_iterator<T>(ss),std::istream_iterator<T>(),std::back_inserter(result));
  return result;
}

template<typename T> std::optional<T> get_joint_attribute(const std::vector<T> attributes, const uint pos) {
  std::optional<T> result;
  try
  {
    result = attributes.at(pos);
  }
  catch (std::out_of_range const& exc)
  {
  }
  return result;
}

class Ax2550HardwareInterface : public hardware_interface::SystemInterface
{
public:
  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

/*
  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
*/

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  std::vector<std::string> position_command_interface_names_;
//  std::vector<std::string> velocity_command_interface_names_;

  std::unique_ptr<ax2550_driver::Ax2550Comms> ax2550comms_;
//  std::vector<ax2550_driver::Joint> joints_;
  Config ax2550_config_;
  // Encoders values last time called
  long enc_left_old_;
  long enc_right_old_;
  double pos_left_old_;
  double pos_right_old_;

  void process_parameters(const hardware_interface::HardwareInfo & info);
};

}  // namespace ax2550_hardware_interface

#endif  // AX2550_HARDWARE_INTERFACE__AX2550_HARDWARE_INTERFACE_HPP_