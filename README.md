# pilsbot_description
Contains the urdf and xacro description files for the Pilsbot model, plus the corresponding meshes.

## xacro arguments

`pilsbot.urdf.xacro` needs `hardware`, there is no default:

- `hardware:=real` selects the robot's `pilsbot_driver/PilsbotDriver` with the serial ports and steering calibration from the robot profile.
- `hardware:=gazebo` selects `gz_ros2_control/GazeboSimSystem` (Gazebo Fortress).

An empty or other value stops xacro with an error naming the argument and the allowed values.

```
xacro $(ros2 pkg prefix --share pilsbot_description)/urdf/pilsbot.urdf.xacro hardware:=real
```

`robot_profile` picks the profile file (default: `profiles/pilsbot.yaml`).
