#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_hw_interface/robot_hw_interface.h>

namespace robot_hw
{
hardware_interface::return_type RobotHardware::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != hardware_interface::return_type::OK) {
    return hardware_interface::return_type::ERROR;
  }

  start_duration_sec_ = stod(info_.hardware_parameters["start_duration_sec"]);
  stop_duration_sec_ = stod(info_.hardware_parameters["stop_duration_sec"]);

  // Create shared memory.
  shm_id = shm::create_shm(&shm_ptr);
  if (shm_id != PROCESS_STATE_NO)
  {
      std::cout << "Create shared memory successfully." << std::endl;
  }
  else
  {
      std::cout << "Create shared memory failed." << std::endl;
  }

  // Create semaphore.
  sem_id = sem::create_semaphore();
  if (sem_id != PROCESS_STATE_NO)
  {
      std::cout << "Create semaphore successfully." << std::endl;
  }
  else
  {
      std::cout << "Create semaphore failed." << std::endl;
  }

  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
RobotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (unsigned int j = 0; j < info_.joints.size(); j++) 
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[j].name, hardware_interface::HW_IF_POSITION, &robot->cur_joint_positions_[j]));
  }
  for (unsigned int j = 0; j < info_.joints.size(); j++) 
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[j].name, hardware_interface::HW_IF_VELOCITY, &robot->cur_joint_velocities_[j]));
  }
  for (unsigned int j = 0; j < info_.joints.size(); j++) 
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[j].name, hardware_interface::HW_IF_EFFORT, &robot->cur_joint_efforts_[j]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RobotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (unsigned int j = 0; j < info_.joints.size(); j++) 
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[j].name, hardware_interface::HW_IF_POSITION, &robot->cmd_joint_positions_[j]));
  }
  for (unsigned int j = 0; j < info_.joints.size(); j++) 
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[j].name, hardware_interface::HW_IF_VELOCITY, &robot->cmd_joint_velocities_[j]));
  }
  for (unsigned int j = 0; j < info_.joints.size(); j++) 
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[j].name, hardware_interface::HW_IF_EFFORT, &robot->cmd_joint_efforts_[j]));
  }

  return command_interfaces;
}


hardware_interface::return_type RobotHardware::start()
{
  RCLCPP_INFO(
    rclcpp::get_logger("RobotHardware"),
    "Starting ...please wait...");

  for (int i = 0; i <= start_duration_sec_; i++) 
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RobotHardware"),
      "%.1f seconds left...", start_duration_sec_ - i);
  }

  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(
    rclcpp::get_logger("RobotHardware"),
    "System Sucessfully started!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotHardware::stop()
{
  RCLCPP_INFO(
    rclcpp::get_logger("RobotHardware"),
    "Stopping ...please wait...");

  for (int i = 0; i <= stop_duration_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RobotHardware"),
      "%.1f seconds left...", stop_duration_sec_ - i);
  }

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(
    rclcpp::get_logger("RobotHardware"),
    "System sucessfully stopped!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotHardware::read()
{
  // Get a semaphore.
  sem::semaphore_p(sem_id);
  for (unsigned int j = 0; j < info_.joints.size(); j++) 
  {
    // read robot states from shared memory to current state variables.
    robot->cur_joint_positions_[j] = shm_ptr->cur_joint_positions_[j];
    robot->cur_joint_velocities_[j] = shm_ptr->cur_joint_velocities_[j];
    robot->cur_joint_efforts_[j] = shm_ptr->cur_joint_efforts_[j];
  }
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotHardware::write()
{
  // read control mode from shared memory.
  // write joint commands to shared memory.
  for (unsigned int j = 0; j < info_.joints.size(); j++) 
  {
    if ((robot->control_mode_[j]=shm_ptr->control_mode_[j]) & robot->position_mode_) 
    {
      shm_ptr->cmd_joint_positions_[j] = robot->cmd_joint_positions_[j];
    }
    else if ((robot->control_mode_[j]=shm_ptr->control_mode_[j]) & robot->velocity_mode_) 
    {
      shm_ptr->cmd_joint_velocities_[j] = robot->cmd_joint_velocities_[j];
    }
    else if ((robot->control_mode_[j]=shm_ptr->control_mode_[j]) & robot->effort_mode_) 
    {
      shm_ptr->cmd_joint_efforts_[j] = robot->cmd_joint_efforts_[j];
    }
  }
  // Release a semaphore.
  sem::semaphore_v(sem_id);
  
  return hardware_interface::return_type::OK;
}

}  // namespace robot_hw

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  robot_hw::RobotHardware,
  hardware_interface::SystemInterface
)
