#include "ax2550_hardware_interface/ax2550_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <iterator>
#include <iostream>

namespace ax2550_hardware_interface
{

hardware_interface::CallbackReturn Ax2550HardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const std::pair<const std::string, std::string>& n : info.hardware_parameters)
  {
    RCLCPP_INFO(rclcpp::get_logger("Ax2550HardwareInterface::on_init"), "key: %s, value: %s", n.first.c_str(), n.second.c_str());
  }
   process_parameters(info);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Ax2550System has exactly two states and two command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Ax2550SystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Ax2550SystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Ax2550SystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Ax2550SystemHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Ax2550SystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

  }

  return CallbackReturn::SUCCESS;
}  

void Ax2550HardwareInterface::process_parameters(const hardware_interface::HardwareInfo & info){
  ax2550_config_.port = info.hardware_parameters.at("ax2550_port");
  std::optional<std::string> cmd_dir_fw = info.hardware_parameters.at("ax2550_cmd_dir_fw");
  ax2550_config_.cmd_dir_fw = cmd_dir_fw.has_value() ? std::move(*cmd_dir_fw) : "!A";
  std::optional<std::string> cmd_dir_bw = info.hardware_parameters.at("ax2550_cmd_dir_bw");
  ax2550_config_.cmd_dir_bw = cmd_dir_bw.has_value() ? std::move(*cmd_dir_bw) : "!a";
  std::optional<std::string> cmd_dir_le = info.hardware_parameters.at("ax2550_cmd_dir_le");
  ax2550_config_.cmd_dir_le = cmd_dir_le.has_value() ? std::move(*cmd_dir_le) : "!B";
  std::optional<std::string> cmd_dir_ri = info.hardware_parameters.at("ax2550_cmd_dir_ri");
  ax2550_config_.cmd_dir_ri = cmd_dir_ri.has_value() ? std::move(*cmd_dir_ri) : "!b";
  std::optional<uint> ticks_per_round = 
    (uint)std::stoul(info.hardware_parameters.at("ax2550_ticks_per_round"),nullptr,0);
  ax2550_config_.ticks_per_round = ticks_per_round.has_value() ? std::move(*ticks_per_round) : 800;
  std::optional<uint> max_a = (uint)std::stoul(info.hardware_parameters.at("ax2550_max_a"),nullptr,0);
  ax2550_config_.max_a = max_a.has_value() ? std::move(*max_a) : 127;
  std::optional<uint> max_b = (uint)std::stoul(info.hardware_parameters.at("ax2550_max_b"),nullptr,0);
  ax2550_config_.max_b = max_b.has_value() ? std::move(*max_b) : 127;
  std::optional<double> speed_constant_x = std::stod(info.hardware_parameters.at("ax2550_speed_constant_x"),nullptr);
  ax2550_config_.speed_constant_x = speed_constant_x.has_value() ? std::move(*speed_constant_x) : 1.0;
  std::optional<double> speed_constant_z = std::stod(info.hardware_parameters.at("ax2550_speed_constant_z"),nullptr);
  ax2550_config_.speed_constant_z = speed_constant_x.has_value() ? std::move(*speed_constant_z) : 1.0;
  std::optional<double> wheel_diameter = std::stod(info.hardware_parameters.at("ax2550_wheel_diameter"),nullptr);
  ax2550_config_.wheel_diameter = speed_constant_x.has_value() ? std::move(*wheel_diameter) : 0.1;
}

hardware_interface::CallbackReturn Ax2550HardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  ax2550comms_ = std::make_unique<ax2550_driver::Ax2550Comms>();
  ax2550comms_->set_device_name(ax2550_config_.port);
  ax2550comms_->set_commands(ax2550_config_.cmd_dir_fw, ax2550_config_.cmd_dir_bw,
    ax2550_config_.cmd_dir_le, ax2550_config_.cmd_dir_ri);
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Ax2550HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Ax2550HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }
  for (auto i = 0u; i < command_interfaces.size(); i++) {
    RCLCPP_INFO(rclcpp::get_logger("Ax2550HardwareInterface::export_command_interfaces"), "command_interfaces[%u].get_full_name: %s",
     i, command_interfaces[i].get_name ().c_str());
  }

  return command_interfaces;
}

hardware_interface::return_type Ax2550HardwareInterface::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // Prepare for new command modes
  for (std::string key : start_interfaces)
  {
    RCLCPP_INFO(rclcpp::get_logger("Ax2550HardwareInterface::prepare_command_mode_switch"), "start_interfaces key: %s", key.c_str());
  }
  // Stop motion on all relevant joints that are stopping
  for (std::string key : stop_interfaces)
  {
    RCLCPP_INFO(rclcpp::get_logger("Ax2550HardwareInterface::prepare_command_mode_switch"), "stop_interfaces key: %s", key.c_str());
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn Ax2550HardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  ax2550comms_->init();
  ax2550comms_->reset_encoders();
  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }
  enc_left_old_ = 0;
  enc_right_old_ = 0;
  pos_left_old_ = 0.0;
  pos_right_old_ = 0.0;
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ax2550HardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ax2550HardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type Ax2550HardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  long enc_left(0), enc_right(0);
  RCLCPP_DEBUG(rclcpp::get_logger("Ax2550HardwareInterface::read"), "Reading...");
  ax2550comms_->query_encoders(enc_left, enc_right);
/*  
  double dleft = (enc_left - this->enc_left_old_) / (double)this->ax2550_config_.ticks_per_meter;
  double dright = (enc_right - this->enc_right_old_) / (double)this->ax2550_config_.ticks_per_meter;
  double vleft = dleft / period.seconds();
  double vright = dright / period.seconds();
  RCLCPP_DEBUG(
    rclcpp::get_logger("Ax2550HardwareInterface::read"), 
    "Got enc_left: %lu, enc_right: %lu!, dleft: %.5f, dright: %.5f, vleft: %.5f, vright: %.5f",
    enc_left, enc_right, dleft, dright, vleft, vright);
*/
  enc_left_old_ = enc_left;
  enc_right_old_ = enc_right;

  // need position in radians
  double pos_left = (enc_left / (double)this->ax2550_config_.ticks_per_round * 2 * M_PI);
  double pos_right = (enc_right / (double)this->ax2550_config_.ticks_per_round * 2 * M_PI);
  // how do we know left and right?
  RCLCPP_DEBUG(
    rclcpp::get_logger("Ax2550HardwareInterface::read"), 
    "enc_left: %lu, pos_left: %.5f,  enc_right %lu, pos_right: %.5f", enc_left, pos_left, enc_right, pos_right);
  hw_positions_[0] = pos_left;
  hw_positions_[1] = pos_right;
  for (uint i = 0; i < hw_positions_.size(); i++)
  {
    RCLCPP_DEBUG(
      rclcpp::get_logger("Ax2550HardwareInterface::read"), "Got hw_position %.5f for joint %d!",
      hw_positions_[i], i);
  }

  // Get current wheel joint positions:
  const double cur_pos_left = pos_left * ax2550_config_.wheel_diameter / 2.0;
  const double cur_pos_right = pos_right * ax2550_config_.wheel_diameter / 2.0;

  // Estimate velocity of wheels using old and current position:
  const double est_vel_left = cur_pos_left - pos_left_old_;
  const double est_vel_right = cur_pos_right - pos_right_old_;

  // Update old position with current:
  pos_left_old_ = cur_pos_left;
  pos_right_old_ = cur_pos_right;

  hw_velocities_[0] = est_vel_left;
  hw_velocities_[1] = est_vel_right;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Ax2550HardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_DEBUG(rclcpp::get_logger("Ax2550HardwareInterface::write"), "Writing...");

  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    // Simulate sending commands to the hardware
    RCLCPP_DEBUG(
      rclcpp::get_logger("Ax2550HardwareInterface::write"), "Got command %.5f for joint %d!",
      hw_commands_[i], i);
  }
  RCLCPP_DEBUG(
    rclcpp::get_logger("Ax2550HardwareInterface::write"), "Joints successfully written!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  // 0 = x, 1 = z
  // Convert mps to rpm
  double A = hw_commands_[0];
  double B = hw_commands_[1];

  double A_rpm = A / (M_PI * ax2550_config_.wheel_diameter) * 60;
  double B_rpm = B / (M_PI * ax2550_config_.wheel_diameter) * 60;

  // Convert rpm to relative
  double A_rel = A_rpm * ax2550_config_.speed_constant_x;
  double B_rel = B_rpm * ax2550_config_.speed_constant_z;

  // Bounds check
  if (A_rel > (double)ax2550_config_.max_a) {
    RCLCPP_DEBUG(rclcpp::get_logger("Ax2550HardwareInterface::write"),
      "cmd_vel_callback: A_rel: %f, ax2550_config_.max_a: %f", A_rel, (double)ax2550_config_.max_a);
    A_rel = (double)ax2550_config_.max_a;
  }
  if (A_rel < -1 * (double)ax2550_config_.max_a) {
    A_rel = -1 * (double)ax2550_config_.max_a;
  }
  if (B_rel > (double)ax2550_config_.max_b) {
    RCLCPP_DEBUG(rclcpp::get_logger("Ax2550HardwareInterface::write"),
      "cmd_vel_callback: B_rel: %f, ax2550_config_.max_b: %f", A_rel, (double)ax2550_config_.max_b);
    B_rel = (double)ax2550_config_.max_b;
  }
  if (B_rel < -1 * (double)ax2550_config_.max_b) {
    B_rel = -1 * (double)ax2550_config_.max_b;
  }
  RCLCPP_DEBUG(rclcpp::get_logger("Ax2550HardwareInterface::write"),
    "cmd_vel_callback: A: %f, Arel: %f, B: %f, Brel: %f", A, A_rel, B, B_rel);

  ax2550comms_->move(A_rel, B_rel);
  return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ax2550_hardware_interface::Ax2550HardwareInterface, hardware_interface::SystemInterface)