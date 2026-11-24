"""Exercise the conversion between calibrated geometry and Gazebo cable inputs."""

import json
import math
from pathlib import Path
from urllib.parse import unquote, urlsplit

import pytest

from ropes_kinematics.model import CableModel
from ropes_simulation.world import SimulationSettings, make_world


SIMULATION = Path(__file__).resolve().parents[1]
DESCRIPTION = SIMULATION.parent / 'ropes_description'
TEMPLATE = SIMULATION / 'worlds' / 'cable_robot.world'
SETTINGS = SIMULATION / 'config' / 'simulation.yaml'
GEOMETRY = DESCRIPTION / 'config' / 'cables.json'


def test_world_uses_cad_and_geometry():
    world = make_world(TEMPLATE, DESCRIPTION, GEOMETRY, SETTINGS)
    model = CableModel.from_file(GEOMETRY)
    settings = SimulationSettings.from_file(SETTINGS)
    for mesh in world.findall('.//mesh/uri'):
        uri = urlsplit(mesh.text)
        assert uri.scheme == 'file' and Path(unquote(uri.path)).is_file()
    assert len(world.findall(".//model[@name='ropes_frame']/link/visual")) >= 7

    payload = world.find(".//model[@name='ropes_payload']")
    assert tuple(map(float, payload.findtext('pose').split()[:3])) == model.home
    assert float(payload.findtext('link/inertial/mass')) == settings.mass_kg
    cable_rows = payload.findall('plugin/cable')
    assert tuple(row.findtext('joint') for row in cable_rows) == model.joint_names
    assert tuple(float(row.findtext('payout_coefficient')) for row in cable_rows) == (
        model.payout_coefficients
    )

    # The unloaded cable lengths preserve static force equilibrium at home.
    net = [0.0, 0.0, -settings.mass_kg * settings.gravity_m_s2]
    for row in cable_rows:
        anchor = tuple(map(float, row.findtext('anchor').split()))
        extension = float(row.findtext('preload_m'))
        length = float(row.findtext('neutral_length'))
        assert length == pytest.approx(math.dist(anchor, model.home))
        for axis, (guide, point) in enumerate(zip(anchor, model.home)):
            net[axis] += settings.stiffness_n_m * extension * (guide - point) / length
    assert net == pytest.approx((0.0, 0.0, 0.0), abs=1e-10)


def test_custom_calibration_changes_virtual_joint_zero(tmp_path):
    config = json.loads(GEOMETRY.read_text(encoding='utf-8'))
    config['home_m'] = [0.01, -0.01, 0.18]
    config_path = tmp_path / 'calibrated.json'
    config_path.write_text(json.dumps(config), encoding='utf-8')
    world = make_world(TEMPLATE, DESCRIPTION, config_path, SETTINGS, use_sim_time=False)
    plugin = world.find(".//plugin[@name='cable_dynamics']")
    assert list(map(float, plugin.findtext('description_home').split())) == config['home_m']
    assert plugin.findtext('use_sim_time') == 'false'


def test_settings_reject_unstable_time_step(tmp_path):
    settings_path = tmp_path / 'settings.yaml'
    settings_path.write_text('physics_step_s: 0.02\n', encoding='utf-8')
    with pytest.raises(ValueError, match='physics_step_s is too large'):
        SimulationSettings.from_file(settings_path)
