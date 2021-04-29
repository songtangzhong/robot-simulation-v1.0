Description of package "gazebo_plugin":

1. This package generates a shared library "libgazebo_plugin.so", 
    which is a gazebo plugin communicating with shared memory.
    To use the shared library, add following line to your file "~/.bashrc":
    export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/franka_robot_ws/install/gazebo_plugin/lib
    The plugin will be used in the package "gazebo_env", file world/panda.world.

2.  "test/test_gazebo_plugin_write.cpp" is used to test the gazebo control plugin
    with writing operation.
    Usage:
    In one terminal:
    gazebo ~/franka_robot_ws/src/gazebo_env/world/panda.world
    In second terminal:
    ros2 run gazebo_plugin test_gazebo_plugin_write

3.  "test/test_gazebo_plugin_read.cpp" is used to test the gazebo control plugin
    with reading operation.
    Usage:
    In one terminal:
    gazebo ~/franka_robot_ws/src/gazebo_env/world/panda.world
    In second terminal:
    ros2 run gazebo_plugin test_gazebo_plugin_read