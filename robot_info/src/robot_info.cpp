#include <robot_info/robot_info.h>

namespace robot_info
{
RobotInfo::RobotInfo()
{
    // Robot arm degree of freedom.
    arm_dof_ = 7;

    // Robot arm joint names.
    arm_joint_names_.resize(arm_dof_);
    arm_joint_names_ = {"panda_joint1","panda_joint2","panda_joint3","panda_joint4",
                        "panda_joint5","panda_joint6","panda_joint7"};
    
    // Resize joint state variables.
    cur_joint_positions_.resize(arm_dof_);
    cur_joint_velocities_.resize(arm_dof_);
    cur_joint_efforts_.resize(arm_dof_);

    // Resize joint command variables.
    cmd_joint_positions_.resize(arm_dof_);
    cmd_joint_velocities_.resize(arm_dof_);
    cmd_joint_efforts_.resize(arm_dof_);

    // Resize control mode variable.
    control_mode_.resize(arm_dof_);

    // Set the initial value.
    for (unsigned int j=0; j<arm_dof_; j++)
    {
        cur_joint_positions_[j] = 0;
        cur_joint_velocities_[j] = 0;
        cur_joint_efforts_[j] = 0;

        cmd_joint_positions_[j] = 0;
        cmd_joint_velocities_[j] = 0;
        cmd_joint_efforts_[j] = 0;

        control_mode_[j] = position_mode_; // default: position mode
    }
    cmd_joint_positions_[3] = cur_joint_positions_[3] = -1.5;
    cmd_joint_positions_[5] = cur_joint_positions_[3] = 1.88;

    // key value of shared memory 
    shm_key_ = 1234;

    // key value of semaphore
    sem_key_ = 2468;
}

RobotInfo::~RobotInfo(){}

} // end namespace robot_info
