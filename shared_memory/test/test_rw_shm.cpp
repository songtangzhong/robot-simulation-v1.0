/*
    This file is used to test the rightness
    of shared memory, including writing and reading.
*/

#include <shared_memory/shared_memory.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    const unsigned int arm_dof = 7;

    // pointer to shared memory.
    shm::Shm *shm_ptr;
    // ID to shared memory.
    int shm_id;

    shm_id = shm::create_shm(&shm_ptr);
    if (shm_id != PROCESS_STATE_NO)
    {
        std::cout << "Create shared memory successfully." << std::endl;
        std::cout << "shm_id: " << shm_id << std::endl;
    }
    else
    {
        std::cout << "Create shared memory failed." << std::endl;
        return 0;
    }

    for (unsigned int j=0; j<arm_dof; j++)
    {
        shm_ptr->cur_joint_positions_[j] = 1;
        shm_ptr->cur_joint_velocities_[j] = 1;
        shm_ptr->cur_joint_efforts_[j] = 1;

        shm_ptr->cmd_joint_positions_[j] = 1;
        shm_ptr->cmd_joint_velocities_[j] = 1;
        shm_ptr->cmd_joint_efforts_[j] = 1;

        shm_ptr->control_mode_[j] = 1;
    }

    rclcpp::Duration(1.0);

    for (unsigned int j=0; j<arm_dof; j++)
    {
        std::cout << "shm_ptr->cur_joint_positions_[" << j << "]: " << shm_ptr->cur_joint_positions_[j] << std::endl;
        std::cout << "shm_ptr->cur_joint_velocities_[" << j << "]: " << shm_ptr->cur_joint_velocities_[j] << std::endl;
        std::cout << "shm_ptr->cur_joint_efforts_[" << j << "]: " << shm_ptr->cur_joint_efforts_[j] << std::endl;

        std::cout << "shm_ptr->cmd_joint_positions_[" << j << "]: " << shm_ptr->cmd_joint_positions_[j] << std::endl;
        std::cout << "shm_ptr->cmd_joint_velocities_[" << j << "]: " << shm_ptr->cmd_joint_velocities_[j] << std::endl;
        std::cout << "shm_ptr->cmd_joint_efforts_[" << j << "]: " << shm_ptr->cmd_joint_efforts_[j] << std::endl;

        std::cout << "shm_ptr->control_mode_[" << j << "]: " << shm_ptr->control_mode_[j] << std::endl;
    }

    if (shm::release_shm(shm_id, &shm_ptr) != PROCESS_STATE_OK)
    {
        std::cout << "Failed to release shared memory." << std::endl;
        return 0;
    }
    else
    {
        std::cout << "Release shared memory successfully." << std::endl;
    }

    return 0;
}