/*
    This file is used to test the rightness
    of shared memory and semaphore.
    It will read data from shared memory for 30 sencods.
    This node must be running after writing operation working immediately.
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

    int sem_id;

    sem_id = sem::create_semaphore();
    if (sem_id != PROCESS_STATE_NO)
    {
        std::cout << "Create semaphore successfully." << std::endl;
        std::cout << "sem_id: " << sem_id << std::endl;
    }
    else
    {
        std::cout << "Create semaphore failed." << std::endl;
        return 0;
    }

    rclcpp::WallRate loop_rate(1000);
    unsigned int i = 0;
    while (rclcpp::ok())
    {
        sem::semaphore_p(sem_id);
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
        sem::semaphore_v(sem_id);
        loop_rate.sleep();
        if (i++ == 1000*30)
        {
            break;
        }
    }

    return 0;
}