"""Tests for the Gazebo Fortress block of pilsbot.urdf.xacro.

`hardware:=gazebo` emits the gz_ros2_control system plugin, wheel friction and
the sensors; `hardware:=real` emits no <gazebo> element at all. Values the
Fortress spike established are in pilsbot/pilsbot_simulation#1.
"""

import pathlib
import shutil
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml

OVERRIDES = '/tmp/pilsbot_profile_test.yaml'


def run_xacro(*args):
    urdf = pathlib.Path(get_package_share_directory('pilsbot_description'),
                        'urdf', 'pilsbot.urdf.xacro')
    return subprocess.run([shutil.which('xacro'), str(urdf), *args],
                          capture_output=True, text=True, timeout=60)


def urdf_of(*args):
    result = run_xacro(*args)
    assert result.returncode == 0, result.stderr
    return result.stdout, ET.fromstring(result.stdout)


@pytest.fixture(scope='module')
def gazebo():
    return urdf_of('hardware:=gazebo', f'controller_overrides:={OVERRIDES}')


@pytest.fixture(scope='module')
def real():
    return urdf_of('hardware:=real')


def test_real_has_no_gazebo_element(real):
    text, robot = real
    assert robot.findall('.//gazebo') == []
    assert '<gazebo' not in text


def test_gazebo_has_the_gz_ros2_control_plugin(gazebo):
    _, robot = gazebo
    plugins = [p for g in robot.findall('gazebo') for p in g.findall('plugin')]
    assert [(p.get('filename'), p.get('name')) for p in plugins] == \
        [('gz_ros2_control-system', 'gz_ros2_control::GazeboSimROS2ControlPlugin')]
    # The robot's controller config first, then the profile overrides with
    # use_sim_time, which the simulation launch writes.
    config = pathlib.Path(get_package_share_directory('pilsbot_control'),
                          'config', 'acker_diff_controller.yaml')
    assert [e.text for e in plugins[0].findall('parameters')] == \
        [str(config), OVERRIDES]


def test_gazebo_without_controller_overrides_stops_xacro():
    # Without the overrides the controllers would run on wall time with the
    # config's geometry instead of the profile's (ADR 0009: no fallback).
    result = run_xacro('hardware:=gazebo')
    assert result.returncode != 0
    assert result.stdout == ''
    assert "argument 'controller_overrides'" in result.stderr.splitlines()[0]


def test_real_does_not_need_controller_overrides(real):
    _, robot = real
    assert robot.find('ros2_control') is not None


def test_no_gazebo_classic_left(gazebo, real):
    for text, _ in (gazebo, real):
        assert 'libgazebo_ros' not in text
        assert 'Gazebo/' not in text
        assert '.so"' not in text
    sources = pathlib.Path(get_package_share_directory('pilsbot_description'),
                           'urdf')
    for source in sources.glob('*.xacro'):
        content = source.read_text()
        assert 'libgazebo_ros' not in content, source
        assert 'Gazebo/' not in content, source


def sensors(robot):
    found = {}
    for g in robot.findall('gazebo'):
        for s in g.findall('sensor'):
            camera = s.find('camera')
            image = camera.find('image')
            found[s.get('name')] = {
                'type': s.get('type'),
                'link': g.get('reference'),
                'topic': s.findtext('topic'),
                'frame_id': s.findtext('ignition_frame_id'),
                'size': (int(image.findtext('width')), int(image.findtext('height'))),
                'format': image.findtext('format'),
                'rate': float(s.findtext('update_rate')),
            }
    return found


def test_gazebo_sensors(gazebo):
    # OAK topics and frames of the Humble pilsbot_depthai mjpeg_node (its
    # defaults: colour 1080p ISP-scaled 2/3 at 30 fps, mono 400p at 15 fps,
    # depth aligned to the left mono camera); the CSI camera as before.
    _, robot = gazebo
    assert sensors(robot) == {
        'oak_rgb': {
            'type': 'camera', 'link': 'oak_rgb_camera_frame',
            'topic': 'camera/color/image_raw',
            'frame_id': 'oak_rgb_camera_optical_frame',
            'size': (1280, 720), 'format': 'R8G8B8', 'rate': 30.0},
        'oak_left': {
            'type': 'camera', 'link': 'oak_left_camera_frame',
            'topic': 'camera/left/image_rect',
            'frame_id': 'oak_left_camera_optical_frame',
            'size': (640, 400), 'format': 'L8', 'rate': 15.0},
        'oak_right': {
            'type': 'camera', 'link': 'oak_right_camera_frame',
            'topic': 'camera/right/image_rect',
            'frame_id': 'oak_right_camera_optical_frame',
            'size': (640, 400), 'format': 'L8', 'rate': 15.0},
        'oak_depth': {
            'type': 'depth_camera', 'link': 'oak_left_camera_frame',
            'topic': 'camera/stereo/depth',
            'frame_id': 'oak_left_camera_optical_frame',
            'size': (640, 400), 'format': 'R_FLOAT32', 'rate': 15.0},
        'front_camera': {
            'type': 'camera', 'link': 'front_camera_link',
            'topic': 'front_camera/image_raw',
            'frame_id': 'front_camera_link',
            'size': (848, 480), 'format': 'R8G8B8', 'rate': 30.0},
    }


def test_sensor_links_and_frames_exist(gazebo):
    _, robot = gazebo
    links = {link.get('name') for link in robot.findall('link')}
    for sensor in sensors(robot).values():
        assert sensor['link'] in links
        assert sensor['frame_id'] in links


def test_front_camera_link_is_in_both_urdfs(gazebo, real):
    # The CSI camera is on the robot, so its frame stays in the real URDF.
    for _, robot in (gazebo, real):
        assert 'front_camera_link' in {link.get('name') for link in robot.findall('link')}


def test_gazebo_wheel_friction(gazebo):
    _, robot = gazebo
    surfaces = {g.get('reference'): (g.findtext('mu1'), g.findtext('mu2'))
                for g in robot.findall('gazebo') if g.find('mu1') is not None}
    assert surfaces == {f'{axle}_wheel_{side}': ('1.0', '1.0')
                        for axle in ('front', 'rear') for side in ('left', 'right')}


def test_pivot_has_no_dynamics(gazebo, real):
    # The spike tracked the steering within 0.001 rad without pivot damping or
    # friction; the real joint stays as measured.
    for _, robot in (gazebo, real):
        joint = [j for j in robot.findall('joint') if j.get('name') == 'steering_axle_joint']
        assert len(joint) == 1
        assert joint[0].find('dynamics') is None


@pytest.mark.parametrize('axle,mass,length', [('front', 4.0, 0.05), ('rear', 1.0, 0.08)])
def test_wheel_inertia_is_a_solid_cylinder(real, axle, mass, length):
    # The 1.0 kg m^2 placeholders turned the chassis when steering at standstill.
    _, robot = real
    profile = yaml.safe_load(pathlib.Path(
        get_package_share_directory('pilsbot_description'),
        'profiles', 'pilsbot.yaml').read_text())
    r = profile['geometry']['wheel_radius']
    for side in ('left', 'right'):
        link = [lk for lk in robot.findall('link') if lk.get('name') == f'{axle}_wheel_{side}'][0]
        inertia = {k: float(v) for k, v in link.find('inertial/inertia').attrib.items()}
        across = mass * (3 * r * r + length * length) / 12
        assert inertia == pytest.approx({
            'ixx': across, 'iyy': across, 'izz': mass * r * r / 2,
            'ixy': 0.0, 'ixz': 0.0, 'iyz': 0.0})
