#include <shared_memory/shared_memory.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

namespace shm
{
// Create shared memory.
/*
    shm_ptr(output): The pointer which stores the shared memory address.
    return: 
        success: shared memory ID.
        fail: PROCESS_STATE_NO
*/
int create_shm(Shm ** shm_ptr)
{
	std::shared_ptr<robot_info::RobotInfo> robot = std::make_shared<robot_info::RobotInfo>();

	// link pointer to shared memory.
    void *shm_ln_ptr = NULL;
    // pointer to shared memory.
    Shm *shm_ptr_;
    // ID to shared memory.
    int shm_id;

    // Create shared memory.
    shm_id = shmget(robot->shm_key_, sizeof(Shm), 0666|IPC_CREAT);
    if (shm_id == -1)
    {
        std::cout << "Failed to create shared memory." << std::endl;
		return PROCESS_STATE_NO;
    }

    // Create the link address of shared memory.
    shm_ln_ptr = shmat(shm_id, 0, 0);
    if (shm_ln_ptr == (void*)-1)
    {
        std::cout << "Failed to create link address of shared memory." << std::endl;
		return PROCESS_STATE_NO;
    }

    // Link the address of shared memory to current address.
    *shm_ptr = shm_ptr_ = (Shm*)shm_ln_ptr;

	return shm_id;
}

// Release shared memory.
/*
    shm_id(input): shared memory ID. 
    shm_ptr(input): shared memory pointer.
    return:
        success: PROCESS_STATE_OK
        fail: PROCESS_STATE_NO
*/
int release_shm(int shm_id, Shm ** shm_ptr)
{
	// Release shared memory.
    if (shmdt(*shm_ptr) == -1)
    {
        std::cout << "Failed to seperate shared memory." << std::endl;
		return PROCESS_STATE_NO;
    }
    if (shmctl(shm_id, IPC_RMID, 0) == -1)
    {
        std::cout << "Failed to release shared memory." << std::endl;
		return PROCESS_STATE_NO;
    }

	return PROCESS_STATE_OK;
}

} // end namespace shm


namespace sem
{
// Create a semaphore.
/*
    return:
        success: semaphore ID
        fail: PROCESS_STATE_NO
*/
int create_semaphore(void)
{
	std::shared_ptr<robot_info::RobotInfo> robot = std::make_shared<robot_info::RobotInfo>();

	// ID of semaphore.
    int sem_id;

	// Create a semaphore.
    sem_id = semget(robot->sem_key_, 1, 0666 | IPC_CREAT);
    if (sem_id == -1)
    {
        std::cout << "Failed to create the semaphore." << std::endl;
		return PROCESS_STATE_NO;
    }

	sem_un sem;
    sem.val = 1;
    if  (semctl(sem_id, 0, SETVAL, sem) == -1)
    {
        std::cout << "Failed to set the value of semaphore." << std::endl;
        return PROCESS_STATE_NO;
    }

    return sem_id;
}

// Delete a semaphore.
/*
    sem_id(input): semaphore ID
    return:
        success: PROCESS_STATE_OK
        fail: PROCESS_STATE_NO
*/
int delete_semaphore(int sem_id)
{
	sem_un sem;

    if (semctl(sem_id, 0, IPC_RMID, sem) == -1)
    {
        std::cout << "Failed to delete the semaphore." << std::endl;
        return PROCESS_STATE_NO;
    }

    return PROCESS_STATE_OK;
}

// Get a semaphore.
/*
    sem_id(input): semaphore ID
    return:
        success: PROCESS_STATE_OK
        fail: PROCESS_STATE_NO
*/
int semaphore_p(int sem_id)
{
	struct sembuf sem_b;
	sem_b.sem_num = 0;
	sem_b.sem_op = -1;
	sem_b.sem_flg = SEM_UNDO;
	if (semop(sem_id, &sem_b, 1) == -1)
	{
		std::cout << "Failed to get the semaphore." << std::endl;
		return PROCESS_STATE_NO;
	}

    return PROCESS_STATE_OK;
}

// Release a semaphore.
/*
    sem_id(input): semaphore ID
    return:
        success: PROCESS_STATE_OK
        fail: PROCESS_STATE_NO
*/
int semaphore_v(int sem_id)
{
	struct sembuf sem_b;
	sem_b.sem_num = 0;
	sem_b.sem_op = 1;
	sem_b.sem_flg = SEM_UNDO;
	if (semop(sem_id, &sem_b, 1) == -1)
	{
		std::cout << "Failed to release the semaphore." << std::endl;
		return PROCESS_STATE_NO;
	}

	return PROCESS_STATE_OK;
}

} // end namespace sem
