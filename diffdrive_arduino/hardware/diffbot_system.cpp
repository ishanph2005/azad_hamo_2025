// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "diffdrive_arduino/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffdrive_arduino
{
hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }


  cfg_.left_front_wheel_name = info_.hardware_parameters["left_front_wheel_name"];
  cfg_.right_front_wheel_name = info_.hardware_parameters["right_front_wheel_name"];
  cfg_.left_back_wheel_name = info_.hardware_parameters["left_back_wheel_name"];
  cfg_.right_back_wheel_name = info_.hardware_parameters["right_back_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  if (info_.hardware_parameters.count("pid_p") > 0)
  {
    cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
    cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
    cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
    cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "PID values not supplied, using defaults.");
  }
  

  wheel_l_f_.setup(cfg_.left_front_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r_f_.setup(cfg_.right_front_wheel_name, cfg_.enc_counts_per_rev);
  wheel_l_b_.setup(cfg_.left_back_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r_b_.setup(cfg_.right_back_wheel_name, cfg_.enc_counts_per_rev);


  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveArduinoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_f_.name, hardware_interface::HW_IF_POSITION, &wheel_l_f_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_f_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_f_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_f_.name, hardware_interface::HW_IF_POSITION, &wheel_r_f_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_f_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_f_.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_l_b_.name, hardware_interface::HW_IF_POSITION, &wheel_l_b_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_l_b_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_b_.vel));
  
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_r_b_.name, hardware_interface::HW_IF_POSITION, &wheel_r_b_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_r_b_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_b_.vel));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveArduinoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_l_f_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_f_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_f_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_f_.cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      wheel_l_b_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_b_.cmd));
  
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      wheel_r_b_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_b_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Configuring ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Cleaning up ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Activating ...please wait...");
  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (cfg_.pid_p > 0)
  {
    comms_.set_pid_values(cfg_.pid_p,cfg_.pid_d,cfg_.pid_i,cfg_.pid_o);
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveArduinoHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  comms_.read_encoder_values(wheel_l_f_.enc, wheel_r_f_.enc, wheel_l_b_.enc, wheel_r_b_.enc);

  double delta_seconds = period.seconds();

  double pos_prev = wheel_l_f_.pos;
  wheel_l_f_.pos = wheel_l_f_.calc_enc_angle();
  wheel_l_f_.vel = (wheel_l_f_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_r_f_.pos;
  wheel_r_f_.pos = wheel_r_f_.calc_enc_angle();
  wheel_r_f_.vel = (wheel_r_f_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_l_b_.pos;
  wheel_l_b_.pos = wheel_l_b_.calc_enc_angle();
  wheel_l_b_.vel = (wheel_l_b_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_r_b_.pos;
  wheel_r_b_.pos = wheel_r_b_.calc_enc_angle();
  wheel_r_b_.vel = (wheel_r_b_.pos - pos_prev) / delta_seconds;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type diffdrive_arduino ::DiffDriveArduinoHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  int motor_l_f_counts_per_loop = wheel_l_f_.cmd / wheel_l_f_.rads_per_count / cfg_.loop_rate;
  int motor_l_b_counts_per_loop = wheel_l_b_.cmd / wheel_l_b_.rads_per_count / cfg_.loop_rate;
  int motor_r_f_counts_per_loop = wheel_r_f_.cmd / wheel_r_f_.rads_per_count / cfg_.loop_rate;
  int motor_r_b_counts_per_loop = wheel_r_b_.cmd / wheel_r_b_.rads_per_count / cfg_.loop_rate;
  // Added averaging od same side encoder data
  int motor_l_counts_per_loop = (motor_l_f_counts_per_loop + motor_l_b_counts_per_loop)/2;
  int motor_r_counts_per_loop = (motor_r_f_counts_per_loop + motor_r_b_counts_per_loop)/2;
  comms_.set_motor_values(motor_l_counts_per_loop, motor_r_counts_per_loop);
  return hardware_interface::return_type::OK;
}

}  // namespace diffdrive_arduino

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_arduino::DiffDriveArduinoHardware, hardware_interface::SystemInterface)
