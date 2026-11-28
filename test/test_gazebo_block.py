"""Tests for the Gazebo Fortress block of pilsbot.urdf.xacro.

`hardware:=gazebo` emits the gz_ros2_control system plugin, wheel friction and
the sensors; `hardware:=real` emits no <gazebo> element at all. Values the
Fortress spike established are in pilsbot/pilsbot_simulation#1.
"""

import math
import pathlib
import shutil
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml

# Never read: xacro only copies the path into the plugin's <parameters>.
OVERRIDES = '/nonexistent/controller_overrides.yaml'


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


def test_controller_overrides_path_is_copied_verbatim():
    # A quote in the path must not break the emptiness check.
    path = "/nonexistent/it's overrides.yaml"
    _, robot = urdf_of('hardware:=gazebo', f'controller_overrides:={path}')
    plugin = robot.find('gazebo/plugin')
    assert plugin.findall('parameters')[-1].text == path


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
            assert s.find('camera/image') is not None, \
                f"sensor {s.get('name')} is not a camera; extend sensors()"
            image = s.find('camera/image')
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
    # depth aligned to the left mono camera); the CSI camera at the robot's
    # jetson_camera output size and rate, in its optical frame.
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
            'frame_id': 'front_camera_optical_frame',
            'size': (1280, 720), 'format': 'R8G8B8', 'rate': 30.0},
    }


def test_no_sensor_has_distortion(gazebo):
    # Fortress's ogre2 does not render lens distortion, so a <distortion>
    # would only make camera_info claim one the pinhole images don't have.
    # Without it the sim's camera_info says D = 0 (pilsbot/pilsbot_simulation#4).
    _, robot = gazebo
    assert robot.findall('.//distortion') == []


# The robot's CSI calibration, jetson_camera camera_info/camera.yaml (the
# Pilsbot's own calibration, jetson_camera a59244b): 1920x1080. jetson_camera
# scales K to its output size, 1280x720 in jetson_camera_launch.py.
CSI_CALIBRATION_SIZE = (1920, 1080)
CSI_CALIBRATION_K = {'fx': 2570.36013, 'fy': 2570.22187, 'cx': 909.89676, 'cy': 586.80859}
CSI_OUTPUT_SIZE = (1280, 720)


def test_csi_intrinsics_match_the_robots_calibration(gazebo):
    _, robot = gazebo
    sensor = [s for g in robot.findall('gazebo') for s in g.findall('sensor')
              if s.get('name') == 'front_camera'][0]
    width = int(sensor.findtext('camera/image/width'))
    height = int(sensor.findtext('camera/image/height'))
    assert (width, height) == CSI_OUTPUT_SIZE
    # Gazebo derives camera_info from the horizontal FOV: fx = fy, centred
    # principal point.
    hfov = float(sensor.findtext('camera/horizontal_fov'))
    fx = width / 2 / math.tan(hfov / 2)
    scale_x = width / CSI_CALIBRATION_SIZE[0]
    scale_y = height / CSI_CALIBRATION_SIZE[1]
    assert fx == pytest.approx(CSI_CALIBRATION_K['fx'] * scale_x, abs=0.01)
    assert fx == pytest.approx(CSI_CALIBRATION_K['fy'] * scale_y, abs=0.5)
    assert sensor.find('camera/lens') is None


def rotation(rpy):
    """URDF rpy (fixed axes x, y, z) as a 3x3 rotation matrix, rows."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def test_front_camera_optical_frame(gazebo, real):
    # REP 103 optical frame on the CSI camera: z forward, x right, y down,
    # in both URDFs (the robot stamps its images with it as well).
    for _, robot in (gazebo, real):
        assert 'front_camera_optical_frame' in {lk.get('name') for lk in robot.findall('link')}
        joints = [j for j in robot.findall('joint')
                  if j.find('child').get('link') == 'front_camera_optical_frame']
        assert len(joints) == 1
        joint = joints[0]
        assert joint.get('type') == 'fixed'
        assert joint.find('parent').get('link') == 'front_camera_link'
        origin = joint.find('origin')
        assert [float(v) for v in origin.get('xyz').split()] == [0.0, 0.0, 0.0]
        matrix = rotation([float(v) for v in origin.get('rpy').split()])
        columns = [[row[i] for row in matrix] for i in range(3)]
        # The optical axes, written in front_camera_link (x forward, y left,
        # z up).
        assert columns[0] == pytest.approx([0.0, -1.0, 0.0], abs=1e-9)  # x: right
        assert columns[1] == pytest.approx([0.0, 0.0, -1.0], abs=1e-9)  # y: down
        assert columns[2] == pytest.approx([1.0, 0.0, 0.0], abs=1e-9)   # z: forward


def test_sensor_links_and_frames_exist(gazebo):
    _, robot = gazebo
    links = {link.get('name') for link in robot.findall('link')}
    for sensor in sensors(robot).values():
        assert sensor['link'] in links
        assert sensor['frame_id'] in links


def test_front_camera_frames_are_in_both_urdfs(gazebo, real):
    # The CSI camera is on the robot, so its frames stay in the real URDF.
    for _, robot in (gazebo, real):
        links = {link.get('name') for link in robot.findall('link')}
        assert {'front_camera_link', 'front_camera_optical_frame'} <= links


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
