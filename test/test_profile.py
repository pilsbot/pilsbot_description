"""Tests for the robot profile loader.

The expected values are the ones in effect on the robot before the profile
existed: the driver's hardcoded ports and steering calibration, and the
controller's geometry.
"""

import pytest

from pilsbot_description import profile


def test_shipped_profile_holds_the_robots_current_values():
    p = profile.load('pilsbot')
    assert p.wheelbase == 0.67
    assert p.front_track == 0.62
    assert p.rear_track == 0.54
    assert p.wheel_radius == 0.125
    assert p.steering_limit == 1.58
    assert p.steering_calibration == [(50702, -1.56601), (32358, 0.0), (14017, 1.56601)]
    assert p.devices == {
        'hoverboard': '/dev/ttyHoverboard',
        'head_mcu': '/dev/ttyHeadMCU',
        'lighting': '/dev/ttyLighting',
    }
    assert p.joystick_mapping == 'ps4'


def test_joystick_is_the_first_gamepad_by_default():
    # Humble's SDL-based joy_node picks a gamepad by index or name, not by device file.
    p = profile.load('pilsbot')
    assert (p.joystick_device_id, p.joystick_device_name) == (0, '')


def test_joystick_can_be_chosen_by_name(tmp_path):
    path = tmp_path / 'named.yaml'
    path.write_text(profile.path_of('pilsbot').read_text().replace(
        'device_id: 0', 'device_id: 1\n    device_name: Wireless Controller'))
    p = profile.load(str(path))
    assert (p.joystick_device_id, p.joystick_device_name) == (1, 'Wireless Controller')


def test_controller_gets_its_geometry_from_the_profile():
    assert profile.load('pilsbot').controller_overrides() == {
        'wheel_separation': 0.62,
        'wheel_radius': 0.125,
        'wheel_base_distance': 0.67,
    }


def test_calibration_is_encoded_for_the_hardware_params():
    assert (profile.load('pilsbot').steering_calibration_param()
            == '50702:-1.56601,32358:0.0,14017:1.56601')


def test_calibration_is_flattened_for_the_head_mcu_node():
    assert (profile.load('pilsbot').steering_calibration_flat()
            == [50702.0, -1.56601, 32358.0, 0.0, 14017.0, 1.56601])


def test_loads_a_profile_by_path(tmp_path):
    path = tmp_path / 'other.yaml'
    path.write_text(profile.path_of('pilsbot').read_text().replace('0.67', '0.7'))
    assert profile.load(str(path)).wheelbase == 0.7
    assert profile.load(str(path)).path == path


def test_missing_key_names_the_key(tmp_path):
    path = tmp_path / 'broken.yaml'
    path.write_text(profile.path_of('pilsbot').read_text().replace('head_mcu:', 'head_mcux:'))
    with pytest.raises(profile.ProfileError, match='devices.head_mcu'):
        profile.load(str(path))


def test_calibration_needs_at_least_two_points(tmp_path):
    path = tmp_path / 'broken.yaml'
    path.write_text(
        'geometry: {wheelbase: 0.67, front_track: 0.62, rear_track: 0.54, wheel_radius: 0.125}\n'
        'steering: {limit: 1.58, calibration: [[32358, 0.0]]}\n'
        'devices: {hoverboard: a, head_mcu: b, lighting: c, joystick: {device_id: 0}}\n'
        'joystick_mapping: ps4\n')
    with pytest.raises(profile.ProfileError, match='steering.calibration'):
        profile.load(str(path))
