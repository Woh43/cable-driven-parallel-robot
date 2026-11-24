import json
import xml.etree.ElementTree as ET

from ropes_kinematics.model import CableModel

from ropes_bringup.description import DEFAULT_GEOMETRY_PATH, robot_description, slider_description


def test_custom_home_is_used_by_live_tf_and_slider_limits():
    config = json.loads(DEFAULT_GEOMETRY_PATH.read_text(encoding='utf-8'))
    config['home_m'] = [0.008, -0.016, 0.19]
    config['height_range_m'] = [0.15, 0.24]
    config['cables'][0]['exit_m'][0] += 0.001
    config['cables'][0]['fixed_path_m'][-1] = config['cables'][0]['exit_m']
    model = CableModel(config)

    robot = ET.fromstring(robot_description(model))
    origin = robot.find("joint[@name='end_effector_x_joint']/origin")
    assert list(map(float, origin.get('xyz').split())) == config['home_m']
    exit_point = robot.find("joint[@name='cable_exit_1_joint']/origin")
    assert list(map(float, exit_point.get('xyz').split())) == config['cables'][0]['exit_m']
    assert all(not link.get('name').startswith('cable_preview_') for link in robot.findall('link'))

    sliders = ET.fromstring(slider_description(model, 'xyz'))
    limits = sliders.find("joint[@name='end_effector_z_joint']/limit")
    assert float(limits.get('lower')) == model.height_range[0] - model.home[2]
    assert float(limits.get('upper')) == model.height_range[1] - model.home[2]
