#ifndef ROBOT_INFO_
#define ROBOT_INFO_

#include <iostream>
#include <vector>

// Define a macro of robot arm dof, 
// which can be used in other header files.
#define ARM_DOF 7

namespace robot_info
{
class RobotInfo
{
public:
    RobotInfo();
    ~RobotInfo();

    // Robot arm degree of freedom.
    unsigned int arm_dof_;

    // Robot arm joint names.
    std::vector<std::string> arm_joint_names_;

    // Store current joint states.
    std::vector<double> cur_joint_positions_;
    std::vector<double> cur_joint_velocities_;
    std::vector<double> cur_joint_efforts_;

    // Store current joint commands.
    std::vector<double> cmd_joint_positions_;
    std::vector<double> cmd_joint_velocities_;
    std::vector<double> cmd_joint_efforts_;

    // Control mode of joints.
    const unsigned int position_mode_ = (1<<0);
    const unsigned int velocity_mode_ = (1<<1);
    const unsigned int effort_mode_ = (1<<2);

    // Stroe the control mode of every joint.
    std::vector<unsigned int> control_mode_;

    // key value of shared memory 
    key_t shm_key_;

    // key value of semaphore
    key_t sem_key_;

}; // end class RobotInfo

} // end namespace robot_info

#endif
