# robot_description
```
This package describes the urdf model of panda robot.
```

## Note:
```
urdf/panda_hw_interface.xacro is the hardware interface corresponding
to ros2 controller manager, which does not work in this package, 
but will works in other packages.
```

## Usage:
1. Run the test file.
```
ros2 launch robot_description test_panda_robot.launch.py
```

## Others:
1. Use the following command to generate a urdf file from xacro file.
```
xacro panda.xacro > panda.urdf
```

2. Use the following command to generate a sdf file from urdf file, which will
    be used in gazebo simulation.
```
gz sdf -p panda.urdf > panda.sdf
Then add "<?xml version="1.0"?>" to the first line in panda.sdf.
```