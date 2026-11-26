"""Robot profile: the single description of one physical Pilsbot.

Launch files load a profile through this module instead of parsing the YAML
themselves; the URDF xacro reads the same file with ``xacro.load_yaml``.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple, Union

import yaml

PACKAGE = 'pilsbot_description'


class ProfileError(ValueError):
    """A profile is missing a key or holds an invalid value."""


def path_of(name: str) -> Path:
    """Path of the profile shipped with this package under the given name."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory(PACKAGE))
    except Exception:  # not installed, e.g. tests run from the source tree
        share = Path(__file__).resolve().parent.parent
    return share / 'profiles' / f'{name}.yaml'


@dataclass(frozen=True)
class Profile:
    path: Path
    wheelbase: float
    front_track: float
    rear_track: float
    wheel_radius: float
    steering_limit: float
    steering_calibration: List[Tuple[int, float]]
    devices: Dict[str, str]
    joystick_mapping: str

    def controller_overrides(self) -> Dict[str, float]:
        """Parameters of the Acker-diff controller that come from the profile."""
        return {
            'wheel_separation': self.front_track,
            'wheel_radius': self.wheel_radius,
            'wheel_base_distance': self.wheelbase,
        }

    def steering_calibration_param(self) -> str:
        """Calibration as the driver's hardware parameter: 'raw:angle,...'."""
        return ','.join(f'{raw}:{angle}' for raw, angle in self.steering_calibration)

    def steering_calibration_flat(self) -> List[float]:
        """Calibration as the head MCU node's flat calibration_val list."""
        return [float(v) for point in self.steering_calibration for v in point]


def load(name_or_path: Union[str, Path]) -> Profile:
    """Load a profile by name (shipped with this package) or by file path."""
    text = str(name_or_path)
    path = Path(text) if ('/' in text or text.endswith('.yaml')) else path_of(text)
    try:
        data = yaml.safe_load(path.read_text())
    except OSError as e:
        raise ProfileError(f'cannot read robot profile {path}: {e}') from e

    def get(dotted, kind):
        node = data
        for key in dotted.split('.'):
            if not isinstance(node, dict) or key not in node:
                raise ProfileError(f'{path}: missing {dotted}')
            node = node[key]
        if kind is float and isinstance(node, int) and not isinstance(node, bool):
            node = float(node)
        if not isinstance(node, kind):
            raise ProfileError(f'{path}: {dotted} must be a {kind.__name__}')
        return node

    calibration = get('steering.calibration', list)
    try:
        points = [(int(raw), float(angle)) for raw, angle in calibration]
    except (TypeError, ValueError) as e:
        raise ProfileError(
            f'{path}: steering.calibration must be [raw, angle] pairs') from e
    if len(points) < 2:
        raise ProfileError(f'{path}: steering.calibration needs at least two points')

    return Profile(
        path=path,
        wheelbase=get('geometry.wheelbase', float),
        front_track=get('geometry.front_track', float),
        rear_track=get('geometry.rear_track', float),
        wheel_radius=get('geometry.wheel_radius', float),
        steering_limit=get('steering.limit', float),
        steering_calibration=points,
        devices={k: get(f'devices.{k}', str)
                 for k in ('hoverboard', 'head_mcu', 'lighting', 'joystick')},
        joystick_mapping=get('joystick_mapping', str),
    )
