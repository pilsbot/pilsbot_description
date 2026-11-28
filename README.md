# pilsbot_description
Contains the urdf and xacro description files for the Pilsbot model, plus the corresponding meshes.

## xacro arguments

`pilsbot.urdf.xacro` needs `hardware`, there is no default:

- `hardware:=real` selects the robot's `pilsbot_driver/PilsbotDriver` with the serial ports and steering calibration from the robot profile.
- `hardware:=gazebo` selects `gz_ros2_control/GazeboSimSystem` (Gazebo Fortress) and adds the `<gazebo>` block from `pilsbot.gazebo.urdf.xacro`: the `gz_ros2_control` system plugin, wheel friction, and the OAK and CSI camera sensors (their gz topic names are listed in that file). It also needs `controller_overrides:=<params file>` with the robot profile's controller geometry and `use_sim_time`, which `pilsbot_driver`'s `pilsbot.launch.py` writes; the plugin loads `pilsbot_control/config/<controller_config>` and then that file.

The `real` URDF has no `<gazebo>` element. Gazebo Classic is not supported.

An empty or other value stops xacro with an error naming the argument and the allowed values.

`cameras:=true|false` (default `true`) matters only with `hardware:=gazebo`: `false` leaves the five camera sensors out of the `<gazebo>` block, so nothing is rendered; the `gz_ros2_control` plugin and everything else stay. The `real` URDF is the same either way. Other values stop xacro.

The CSI camera's images are in `front_camera_optical_frame` (REP 103 optical frame under `front_camera_link`: z forward, x right, y down), on the robot and in the sim. The sim's CSI sensor is 1280×720 at 30 fps with the field of view of the robot's calibration (`jetson_camera` `camera_info/camera.yaml`, scaled to 1280×720) and no distortion: Fortress's ogre2 does not render lens distortion, so its `camera_info` says D = 0 to match its pinhole images.

```
xacro $(ros2 pkg prefix --share pilsbot_description)/urdf/pilsbot.urdf.xacro hardware:=real
```

`robot_profile` picks the profile file (default: `profiles/pilsbot.yaml`).
