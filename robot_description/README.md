Description of package "robot_description":

1. This package descripes the urdf model of panda robot.

2.(Note) "urdf/panda_hw_interface.xacro" is the hardware interface associated
    with ros2 controller manager, which does not work in this package, but will
    works in future package.

3. Usage:
    ros2 launch robot_description test_panda_robot.launch.py

4. Use the following command to generate a urdf file from xacro file.
    xacro panda.xacro > panda.urdf

5. Use the following command to generate a sdf file from urdf file, which will
    be used in gazebo simulation.
    gz sdf -p panda.urdf > panda.sdf
    Then add "<?xml version="1.0"?>" to the first line in panda.sdf.