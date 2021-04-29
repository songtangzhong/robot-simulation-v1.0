/*
    This file is used to test the rightness
    of semaphore.
*/

#include <shared_memory/shared_memory.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

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

    if (sem::semaphore_p(sem_id) == PROCESS_STATE_OK)
    {
        std::cout << "Get a semaphore successfully." << std::endl;
    }
    else
    {
        std::cout << "Get a semaphore failed." << std::endl;
        return 0;
    }

    rclcpp::Duration(1.0);

    if (sem::semaphore_v(sem_id) == PROCESS_STATE_OK)
    {
        std::cout << "Release a semaphore successfully." << std::endl;
    }
    else
    {
        std::cout << "Release a semaphore failed." << std::endl;
        return 0;
    }

    if (sem::delete_semaphore(sem_id) == PROCESS_STATE_OK)
    {
        std::cout << "Delete semaphore successfully." << std::endl;
    }
    else
    {
        std::cout << "Delete semaphore failed." << std::endl;
        return 0;
    }

    return 0;
}