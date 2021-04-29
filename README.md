# Illustratin of packages:

# robot_description
1. This package includes descriptions of a robot, 
    mainly are ".xacro" files in "./urdf". The robot 
    model is described in urdf form.

# gazebo_env
1. This package defines the gazebo simulation model and environment.

# robot_info
1. This package defines basic information of a robot.
    It can be used in other packages in the form of
    dynamic library.

# shared_memory
1. This package defines the shared memory operation, including
    writting and reading of shared memory, and using method of 
    semaphore.

# gazebo_plugin
1. This package defines gazebo plugin to operate the robot in gazebo,
    including reading and writing. The plugin will communicate with shared
    memory.

# robot_hw_interface
1. This package realizes the function of robot hardware interface
    and controler manager. It will communicate with shared memory.