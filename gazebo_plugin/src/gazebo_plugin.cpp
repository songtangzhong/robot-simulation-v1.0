#include <gazebo_plugin/gazebo_plugin.h>

namespace gazebo_plugin
{
ControlPlugin::ControlPlugin()
{
    std::cout << "Start Robot Simulation ..." << std::endl;

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

    // Set the initial values of shared memory.
    for (unsigned int j=0; j< robot->arm_dof_; j++)
    {
        shm_ptr->cur_joint_positions_[j] = robot->cur_joint_positions_[j];
        shm_ptr->cur_joint_velocities_[j] = robot->cur_joint_velocities_[j];
        shm_ptr->cur_joint_efforts_[j] = robot->cur_joint_efforts_[j];

        shm_ptr->cmd_joint_positions_[j] = robot->cmd_joint_positions_[j];
        shm_ptr->cmd_joint_velocities_[j] = robot->cmd_joint_velocities_[j];
        shm_ptr->cmd_joint_efforts_[j] = robot->cmd_joint_efforts_[j];

        shm_ptr->control_mode_[j] = robot->control_mode_[j];
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
}

ControlPlugin::~ControlPlugin()
{ 
    // Delete semaphore.
    if (sem::delete_semaphore(sem_id) == PROCESS_STATE_OK)
    {
        std::cout << "Delete semaphore successfully." << std::endl;
    }
    else
    {
        std::cout << "Delete semaphore failed." << std::endl;
    }

    // Release shared memory.
    if (shm::release_shm(shm_id, &shm_ptr) == PROCESS_STATE_OK)
    {
        std::cout << "Release shared memory successfully." << std::endl;
    }
    else
    {
        std::cout << "Release shared memory failed." << std::endl;
    }

    std::cout << "Simulation has been finished successfully by user." << std::endl;
}

// Overloaded Gazebo entry point
void ControlPlugin::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  std::cout << "Load control plugin ..." << std::endl;

  // Save pointers to the model
  parent_model_ = parent;

  // Get the Gazebo simulation rate
  double sim_rate = parent_model_->GetWorld()->Physics()->GetRealTimeUpdateRate();
  std::cout << "Simulation rate: " << sim_rate << " Hz." << std::endl;

  // Get joint pointer from model
  for (unsigned int j=0; j< robot->arm_dof_; j++)
  {
     gazebo::physics::JointPtr simjoint = parent_model_->GetJoint(robot->arm_joint_names_[j]);
     arm_joints_.push_back(simjoint);
  }

  // Listen to the update event. This event is broadcast every simulation iteration.
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    boost::bind(&ControlPlugin::Update, this));

  std::cout << "Load control plugin successfully." << std::endl;
}

// Called by the world update start event
void ControlPlugin::Update()
{
  /* Note: 
     We must do writing operation, and then do reading operation in this function.
     Also, we must do reading operation, and then do writing operation in ROS2 controller manager.
     Doing this can form a closed-loop system.
  */

  // Get a semaphore.
  sem::semaphore_p(sem_id);
  for (unsigned int j=0; j< robot->arm_dof_; j++)
  {
    // Read control mode and commands from shared memory, then write them into gazebo.
    if ((robot->control_mode_[j]=shm_ptr->control_mode_[j]) & robot->position_mode_)
    {
      arm_joints_[j]->SetPosition(0, robot->cmd_joint_positions_[j]=shm_ptr->cmd_joint_positions_[j]);
    }
    else if ((robot->control_mode_[j]=shm_ptr->control_mode_[j]) & robot->velocity_mode_)
    {
      arm_joints_[j]->SetVelocity(0, robot->cmd_joint_velocities_[j]=shm_ptr->cmd_joint_velocities_[j]);
    }
    else if ((robot->control_mode_[j]=shm_ptr->control_mode_[j]) & robot->effort_mode_)
    {
      arm_joints_[j]->SetForce(0, robot->cmd_joint_efforts_[j]=shm_ptr->cmd_joint_efforts_[j]);
    }
    
    // Read current robot states from gazebo, then write them into shared memory.
    shm_ptr->cur_joint_positions_[j] = robot->cur_joint_positions_[j] = arm_joints_[j]->Position(0);
    shm_ptr->cur_joint_velocities_[j] = robot->cur_joint_velocities_[j] = arm_joints_[j]->GetVelocity(0);
    shm_ptr->cur_joint_efforts_[j] = robot->cur_joint_efforts_[j] = arm_joints_[j]->GetForce(0u);
  }
  // Release a semaphore.
  sem::semaphore_v(sem_id);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ControlPlugin)

} // end namespace gazebo_plugin
