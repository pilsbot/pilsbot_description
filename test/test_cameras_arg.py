"""Tests for the `cameras` xacro argument of pilsbot.urdf.xacro.

`cameras:=false` leaves the camera sensors out of the Gazebo block, so
nothing renders and control tests run faster (pilsbot/pilsbot_vscode_ws#35).
Everything else in the URDF stays, and the real robot's URDF does not change.
"""

import pathlib
import shutil
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest

# Never read: xacro only copies the path into the plugin's <parameters>.
GAZEBO = ('hardware:=gazebo', 'controller_overrides:=/nonexistent/controller_overrides.yaml')


def run_xacro(*args):
    urdf = pathlib.Path(get_package_share_directory('pilsbot_description'),
                        'urdf', 'pilsbot.urdf.xacro')
    return subprocess.run([shutil.which('xacro'), str(urdf), *args],
                          capture_output=True, text=True, timeout=60)


def urdf_of(*args):
    result = run_xacro(*args)
    assert result.returncode == 0, result.stderr
    return result.stdout


def canonical(robot):
    return ET.canonicalize(ET.tostring(robot, encoding='unicode'), strip_text=True)


def without_sensors(text):
    """The URDF with every <gazebo> element that holds a <sensor> removed."""
    robot = ET.fromstring(text)
    for gazebo in robot.findall('gazebo'):
        if gazebo.find('sensor') is not None:
            robot.remove(gazebo)
    return canonical(robot)


def test_cameras_false_has_no_sensor():
    text = urdf_of(*GAZEBO, 'cameras:=false')
    assert ET.fromstring(text).findall('.//sensor') == []
    assert '<sensor' not in text


def test_cameras_false_keeps_everything_else():
    # The gz_ros2_control plugin, the wheel friction, links and joints stay.
    with_cameras = urdf_of(*GAZEBO, 'cameras:=true')
    no_cameras = urdf_of(*GAZEBO, 'cameras:=false')
    assert without_sensors(with_cameras) == canonical(ET.fromstring(no_cameras))
    plugins = [p.get('filename') for p in ET.fromstring(no_cameras).findall('gazebo/plugin')]
    assert plugins == ['gz_ros2_control-system']


def test_cameras_true_has_the_five_sensors():
    robot = ET.fromstring(urdf_of(*GAZEBO, 'cameras:=true'))
    names = [s.get('name') for g in robot.findall('gazebo') for s in g.findall('sensor')]
    assert sorted(names) == ['front_camera', 'oak_depth', 'oak_left', 'oak_rgb', 'oak_right']


def test_cameras_default_is_true():
    assert urdf_of(*GAZEBO) == urdf_of(*GAZEBO, 'cameras:=true')


@pytest.mark.parametrize('value', ['true', 'false'])
def test_real_urdf_does_not_depend_on_cameras(value):
    assert urdf_of('hardware:=real', f'cameras:={value}') == urdf_of('hardware:=real')


@pytest.mark.parametrize('hardware', [GAZEBO, ('hardware:=real',)])
@pytest.mark.parametrize('value', ['TRUE', 'off', 'yes', '1', '0', 'none'])
def test_unknown_cameras_value_stops_xacro(hardware, value):
    # ADR 0009: a typo must not silently pick one of the two.
    result = run_xacro(*hardware, f'cameras:={value}')
    assert result.returncode != 0
    assert result.stdout == ''
    headline = result.stderr.splitlines()[0]
    assert "argument 'cameras' must be 'true' or 'false'" in headline
    assert value in headline
