"""Check pulse rounding against real geometry and calibrated actuator travel."""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import pytest
from ropes_kinematics.model import CableModel
import yaml

from ropes_hardware.motor_targets import quantized_motor_target
from ropes_hardware.settings import HardwareSettings


@pytest.fixture
def model_and_config():
    geometry = Path(get_package_share_directory('ropes_description')) / 'config/cables.json'
    settings = Path(get_package_share_directory('ropes_hardware')) / 'config/hardware.example.yaml'
    model = CableModel.from_file(geometry)
    return model, yaml.safe_load(settings.read_text(encoding='utf-8'))


def load(config, model, tmp_path):
    path = tmp_path / 'settings.yaml'
    path.write_text(yaml.safe_dump(config), encoding='utf-8')
    return HardwareSettings.from_file(path, model.joint_names)


def test_round_inwards_if_nearest_pulse_exceeds_calibrated_limit(model_and_config, tmp_path):
    model, config = model_and_config
    target = model.inverse((0.0, 0.0, 0.15))
    config['axes'][model.joint_names[0]]['upper_rad'] = target[0]
    settings = load(config, model, tmp_path)
    steps = tuple(math.tau / axis.pulses_per_revolution for axis in settings.axes)
    home = (0.0,) * 3
    assert round(target[0] / steps[0]) * steps[0] > target[0]

    motors = quantized_motor_target(model, settings, target, target, home, steps)
    assert motors[0] == math.floor(target[0] / steps[0]) * steps[0]
    achieved = settings.check_drums(settings.to_drum_angles(motors, home))
    assert math.dist(model.forward(achieved), (0.0, 0.0, 0.15)) < 0.001


def test_fails_safely_when_all_neighboring_pulses_exceed_travel(model_and_config, tmp_path):
    model, config = model_and_config
    for axis in config['axes'].values():
        axis.update(lower_rad=-0.0002, upper_rad=0.0002)
    settings = load(config, model, tmp_path)
    steps = tuple(math.tau / axis.pulses_per_revolution for axis in settings.axes)
    home = tuple(0.5 * step for step in steps)

    with pytest.raises(ValueError, match='No adjacent motor pulse target'):
        quantized_motor_target(model, settings, (0.0,) * 3, (0.0,) * 3, home, steps)
