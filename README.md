# pilsbot_description
Contains the urdf and xacro description files for the Pilsbot model, plus the corresponding meshes.

## xacro arguments

`pilsbot.urdf.xacro` needs `hardware`, there is no default:

- `hardware:=real` selects the robot's `pilsbot_driver/PilsbotDriver` with the serial ports and steering calibration from the robot profile.
- `hardware:=gazebo` selects `gz_ros2_control/GazeboSimSystem` (Gazebo Fortress) and adds the `<gazebo>` block from `pilsbot.gazebo.urdf.xacro`: the `gz_ros2_control` system plugin, wheel friction, and the OAK and CSI camera sensors (their gz topic names are listed in that file). It also needs `controller_overrides:=<params file>` with the robot profile's controller geometry and `use_sim_time`, which `pilsbot_driver`'s `pilsbot.launch.py` writes; the plugin loads `pilsbot_control/config/<controller_config>` and then that file.

The `real` URDF has no `<gazebo>` element. Gazebo Classic is not supported.

An empty or other value stops xacro with an error naming the argument and the allowed values.

```
xacro $(ros2 pkg prefix --share pilsbot_description)/urdf/pilsbot.urdf.xacro hardware:=real
```

`robot_profile` picks the profile file (default: `profiles/pilsbot.yaml`).
