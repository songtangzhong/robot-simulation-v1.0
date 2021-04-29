#ifndef SHARED_MEMORY_H_
#define SHARED_MEMORY_H_

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <iostream>
#include <robot_info/robot_info.h>

// Define process state in shared memory operation.
#define PROCESS_STATE_OK  1
#define PROCESS_STATE_NO -1

namespace shm
{
typedef struct
{
    // Store current joint states.
    double cur_joint_positions_[ARM_DOF];
    double cur_joint_velocities_[ARM_DOF];
    double cur_joint_efforts_[ARM_DOF];

    // Store current joint commands.
    double cmd_joint_positions_[ARM_DOF];
    double cmd_joint_velocities_[ARM_DOF];
    double cmd_joint_efforts_[ARM_DOF];

    // Store joint control mode.
    unsigned int control_mode_[ARM_DOF];
}Shm;

// Create shared memory.
/*
    shm_ptr(output): The pointer which stores the shared memory address.
    return: 
        success: shared memory ID.
        fail: PROCESS_STATE_NO
*/
int create_shm(Shm ** shm_ptr);

// Release shared memory.
/*
    shm_id(input): shared memory ID. 
    shm_ptr(input): shared memory pointer.
    return:
        success: PROCESS_STATE_OK
        fail: PROCESS_STATE_NO
*/
int release_shm(int shm_id, Shm ** shm_ptr);

} // end namespace shm


namespace sem
{
// parameter union
typedef union 
{
	int val;
	struct semid_ds *buf;
	unsigned short *arry;
}sem_un;

// Create a semaphore.
/*
    return:
        success: semaphore ID
        fail: PROCESS_STATE_NO
*/
int create_semaphore(void);

// Delete a semaphore.
/*
    sem_id(input): semaphore ID
    return:
        success: PROCESS_STATE_OK
        fail: PROCESS_STATE_NO
*/
int delete_semaphore(int sem_id);

// Get a semaphore.
/*
    sem_id(input): semaphore ID
    return:
        success: PROCESS_STATE_OK
        fail: PROCESS_STATE_NO
*/
int semaphore_p(int sem_id);

// Release a semaphore.
/*
    sem_id(input): semaphore ID
    return:
        success: PROCESS_STATE_OK
        fail: PROCESS_STATE_NO
*/
int semaphore_v(int sem_id);

} // end namespace sem

#endif
