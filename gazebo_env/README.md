Description of package "gazebo_env":

1. Folder "panda" includes the sdf description of panda robot.
    To use the panda model, add the following line to your "~/.bashrc":
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/franka_robot_ws/src/gazebo_env

2. Usage:
    gazebo ~/franka_robot_ws/src/gazebo_env/world/panda.world