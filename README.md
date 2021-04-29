# robot_sim
    This is an open robot project, including simulation and control.
The robot is franka panda, which has been simulated in gazebo11. The
whole project is realized based on ros2-foxy.

## robot_description
    This package includes descriptions of a robot, 
mainly are ".xacro" files in "./urdf". The robot 
model is described in urdf form.

## gazebo_env
    This package defines the gazebo simulation model and environment.

3. robot_info

    This package defines basic information of a robot.
It can be used in other packages in the form of
dynamic library.

4. shared_memory

    This package defines the shared memory operation, including
writting and reading of shared memory, and using method of 
semaphore.

5. gazebo_plugin

    This package defines gazebo plugin to operate the robot in gazebo,
including reading and writing. The plugin will communicate with shared
memory.

6. robot_hw_interface

    This package realizes the function of robot hardware interface
and controler manager. It will communicate with shared memory.