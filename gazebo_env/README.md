# gazebo_env

1. Description:
```
./panda includes the sdf description of panda robot.
To use the panda model, add the following command to your ~/.bashrc:
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/franka_robot_ws/src/gazebo_env
```

2. Usage:
```
gazebo ~/franka_robot_ws/src/gazebo_env/world/panda.world
This should be run in firstly and shutdown in finally in the whole operation of the project.
```
