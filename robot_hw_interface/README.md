Description of package "robot_hw_interface":

1. This package generates a robot plugin "librobot_hw_interface.so"
    and a shared library "libcontroller_configure.so". To use the shared
    library, the path of the library and header files should be provided.

2. Usage:
    (1) gazebo ~/franka_robot_ws/src/gazebo_env/world/panda.world
    (2) ros2 launch robot_hw_interface robot_hw_interface.launch.py

3. "test/test_switch_controller.cpp" is used to test the function of switching
    controllers.
    Usage:
    ros2 run robot_hw_interface test_switch_controller [start_controller] [stop_controller]
