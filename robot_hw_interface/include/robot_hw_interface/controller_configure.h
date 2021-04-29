#ifndef CONTROLLER_CONFIGURE_H
#define CONTROLLER_CONFIGURE_H

#include <chrono>
#include <memory>
#include <string>
#include <controller_manager_msgs/srv/load_start_controller.hpp>
#include <controller_manager_msgs/srv/load_configure_controller.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_info/robot_info.h>
#include <shared_memory/shared_memory.h>
#include <iostream>

namespace controller_configure
{
class ControllerConfigure
{
public:
    std::shared_ptr<rclcpp::Node> nh_;

    ControllerConfigure(const std::string & node_name);
    ~ControllerConfigure();

    void load_start_controller(const std::string & controller_name);

    void load_configure_controller(const std::string & controller_name);

    void switch_controller(const std::string & start_controller, const std::string & stop_controller);

private:
    rclcpp::Client<controller_manager_msgs::srv::LoadStartController>::SharedPtr load_start_controller_cli_;
    rclcpp::Client<controller_manager_msgs::srv::LoadConfigureController>::SharedPtr load_configure_controller_cli_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_cli_;

    // store robot information
    std::shared_ptr<robot_info::RobotInfo> robot = std::make_shared<robot_info::RobotInfo>();

    // pointer to shared memory.
    shm::Shm *shm_ptr;
    // ID to shared memory.
    int shm_id;

    // ID to semaphore.
    int sem_id;
};

} // end namespace controller_configure

#endif
