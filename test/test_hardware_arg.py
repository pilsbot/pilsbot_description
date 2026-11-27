"""Tests for the required `hardware` xacro argument of pilsbot.urdf.xacro.

They run the xacro command line the way the launch files do, against the
installed package (the URDF includes its parts with $(find ...)).

golden/ros2_control_real.xml is the <ros2_control> block the robot got
before this argument existed (`use_fake_hardware:=false`).
"""

import pathlib
import re
import shutil
import subprocess

from ament_index_python.packages import get_package_share_directory
import pytest

GOLDEN = pathlib.Path(__file__).parent / 'golden'


def run_xacro(*args):
    urdf = pathlib.Path(get_package_share_directory('pilsbot_description'),
                        'urdf', 'pilsbot.urdf.xacro')
    return subprocess.run([shutil.which('xacro'), str(urdf), *args],
                          capture_output=True, text=True, timeout=60)


def ros2_control_block(urdf):
    match = re.search(r'^ *<ros2_control .*?</ros2_control>\n', urdf, re.S | re.M)
    assert match, 'no <ros2_control> block in the URDF'
    return match.group(0)


def test_real_keeps_the_robots_ros2_control_block():
    result = run_xacro('hardware:=real')
    assert result.returncode == 0, result.stderr
    assert ros2_control_block(result.stdout) == \
        (GOLDEN / 'ros2_control_real.xml').read_text()


def test_gazebo_selects_the_fortress_system_plugin():
    result = run_xacro('hardware:=gazebo')
    assert result.returncode == 0, result.stderr
    block = ros2_control_block(result.stdout)
    plugins = re.findall(r'<plugin>(.*?)</plugin>', block)
    assert plugins == ['gz_ros2_control/GazeboSimSystem']
    # The driver's serial ports and calibration are not hardware params in sim.
    assert '<param name="hoverboard_tty_device">' not in block
    # Same joints and interfaces as on the robot: the controllers don't change.
    real = (GOLDEN / 'ros2_control_real.xml').read_text()
    assert re.findall(r'<joint .*?</joint>', block, re.S) == \
        re.findall(r'<joint .*?</joint>', real, re.S)


def test_gazebo_has_no_driver_sensors():
    # hoverboard_api and head_mcu are state interfaces of PilsbotDriver;
    # GazeboSimSystem does not export them.
    block = ros2_control_block(run_xacro('hardware:=gazebo').stdout)
    assert '<sensor' not in block


@pytest.mark.parametrize('args', [(), ('hardware:=',), ('hardware:=fake',),
                                  ('hardware:=Real',)])
def test_missing_or_unknown_hardware_stops_xacro(args):
    result = run_xacro(*args)
    assert result.returncode != 0
    assert result.stdout == ''
    message = result.stderr
    assert "'hardware'" in message
    assert "'real'" in message and "'gazebo'" in message


def test_use_fake_hardware_is_gone():
    # Undeclared xacro arguments are ignored, so the old flag selects nothing.
    result = run_xacro('hardware:=real', 'use_fake_hardware:=true')
    assert result.returncode == 0, result.stderr
    assert ros2_control_block(result.stdout) == \
        (GOLDEN / 'ros2_control_real.xml').read_text()
    for name in ('pilsbot.urdf.xacro', 'pilsbot.ros2_control.urdf.xacro'):
        source = pathlib.Path(get_package_share_directory('pilsbot_description'),
                              'urdf', name).read_text()
        assert 'use_fake_hardware' not in source
    assert 'GazeboSystem<' not in result.stdout
