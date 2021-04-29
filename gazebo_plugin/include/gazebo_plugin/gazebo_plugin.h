#ifndef GAZEBO_PLUGIN_H
#define GAZEBO_PLUGIN_H

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/Joint.hh>
#include <robot_info/robot_info.h>
#include <shared_memory/shared_memory.h>
#include <iostream>

namespace gazebo_plugin
{
class ControlPlugin : public gazebo::ModelPlugin
{
public:
    ControlPlugin();
    ~ControlPlugin();

    // Overloaded Gazebo entry point
    void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);

    // Called by the world update start event
    void Update();

private:
    // Pointer to the model
    gazebo::physics::ModelPtr parent_model_;

    // Pointer to the arm joints
    std::vector<gazebo::physics::JointPtr> arm_joints_;

    // Pointer to the update event connection
    gazebo::event::ConnectionPtr update_connection_;

    // Store the information of robot.
    std::shared_ptr<robot_info::RobotInfo> robot = std::make_shared<robot_info::RobotInfo>();

    // pointer to shared memory.
    shm::Shm *shm_ptr;
    // ID to shared memory.
    int shm_id;
    
    // ID to semaphore.
    int sem_id;
}; // end class ControlPlugin

} // end namespace gazebo_plugin

#endif
