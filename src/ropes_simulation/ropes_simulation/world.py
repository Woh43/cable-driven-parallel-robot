"""Assemble the simulation world from the CAD description and calibrated rope geometry."""

from copy import deepcopy
from dataclasses import dataclass
from itertools import pairwise
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

from ropes_kinematics.model import CableModel


CARTESIAN_JOINTS = ('end_effector_x_joint', 'end_effector_y_joint', 'end_effector_z_joint')


@dataclass(frozen=True)
class SimulationSettings:
    mass_kg: float = 0.12
    stiffness_n_m: float = 1200.0
    damping_ns_m: float = 10.0
    max_drum_speed_rad_s: float = 6.0
    state_rate_hz: float = 50.0
    physics_step_s: float = 0.001
    gravity_m_s2: float = 9.81

    @classmethod
    def from_file(cls, path):
        try:
            settings = cls(**yaml.safe_load(Path(path).read_text(encoding='utf-8')))
        except (TypeError, yaml.YAMLError) as error:
            raise ValueError(f'Invalid simulation settings file {path}: {error}') from error
        for name, value in vars(settings).items():
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value)):
                raise ValueError(f'{name} must be a finite number')
            if value < 0.0 or (name != 'damping_ns_m' and value == 0.0):
                raise ValueError(f'{name} must be positive (damping may be zero)')
        # Leave a margin below the explicit integration stability limit for three taut cables.
        largest_frequency = math.sqrt(3.0 * settings.stiffness_n_m / settings.mass_kg)
        largest_decay = 3.0 * settings.damping_ns_m / settings.mass_kg
        if settings.physics_step_s * max(largest_frequency, largest_decay) > 0.5:
            raise ValueError('physics_step_s is too large for this mass, stiffness and damping')
        if settings.state_rate_hz * settings.physics_step_s > 1.0:
            raise ValueError('state_rate_hz must fit within the physics step rate')
        return settings


def _text(values):
    return ' '.join(f'{value:.15g}' for value in values)


def _add(parent, tag, text=None, **attributes):
    element = ET.SubElement(parent, tag, attributes)
    if text is not None:
        element.text = str(text)
    return element


def _urdf_pose(element):
    origin = element.find('origin')
    if origin is None:
        return (0.0,) * 6
    return tuple(float(value) for field in ('xyz', 'rpy')
                 for value in origin.get(field, '0 0 0').split())


def _material(visual, rgba):
    material = _add(visual, 'material')
    _add(material, 'ambient', rgba)
    _add(material, 'diffuse', rgba)


def _copy_geometry(robot, parent, source, tag, description_share):
    for index, original in enumerate(source.findall(tag)):
        element = _add(parent, tag, name=f'{tag}_{index}')
        _add(element, 'pose', _text(_urdf_pose(original)))
        geometry = deepcopy(original.find('geometry'))
        mesh = geometry.find('mesh')
        if mesh is not None:
            uri = mesh.attrib.pop('filename')
            prefix = 'package://ropes_description/'
            if uri.startswith(prefix):
                uri = (description_share / uri[len(prefix):]).resolve(strict=True).as_uri()
            _add(mesh, 'uri', uri)
            if 'scale' in mesh.attrib:
                _add(mesh, 'scale', mesh.attrib.pop('scale'))
        sphere = geometry.find('sphere')
        if sphere is not None:
            _add(sphere, 'radius', sphere.attrib.pop('radius'))
        element.append(geometry)
        if tag == 'visual':
            name = original.find('material').get('name')
            rgba = robot.find(f"material[@name='{name}']/color").get('rgba')
            _material(element, rgba)


def _cable_cylinder(link, name, start, end, radius, rgb):
    vector = tuple(b - a for a, b in zip(start, end))
    center = tuple((a + b) * 0.5 for a, b in zip(start, end))
    length = math.hypot(*vector)
    if length < 1e-10:
        raise ValueError(f'{name} has coincident cable geometry points')
    horizontal = math.hypot(*vector[:2])
    orientation = (0.0, math.atan2(horizontal, vector[2]), math.atan2(vector[1], vector[0]))
    visual = _add(link, 'visual', name=name)
    _add(visual, 'pose', _text(center + orientation))
    cylinder = _add(_add(visual, 'geometry'), 'cylinder')
    _add(cylinder, 'radius', radius)
    _add(cylinder, 'length', length)
    _add(visual, 'cast_shadows', 'false')
    _material(visual, _text((*rgb, 1.0)))


def make_world(template, description_share, config_path, settings_path, use_sim_time=True):
    """Return a complete SDF world with a payload, frame, drums and calibrated plugin data."""
    tree = ET.parse(template)
    world = tree.getroot().find('world')
    description_share = Path(description_share)
    robot = ET.parse(description_share / 'urdf' / 'cable_robot.urdf').getroot()
    settings = SimulationSettings.from_file(settings_path)
    model = CableModel.from_file(config_path)
    world.find('gravity').text = _text((0.0, 0.0, -settings.gravity_m_s2))
    world.find('physics/max_step_size').text = str(settings.physics_step_s)
    world.find('physics/real_time_update_rate').text = str(1.0 / settings.physics_step_s)

    frame = _add(world, 'model', name='ropes_frame')
    _add(frame, 'static', 'true')
    frame_link = _add(frame, 'link', name='base_link')
    base = robot.find("link[@name='base_link']")
    for tag in ('visual', 'collision'):
        _copy_geometry(robot, frame_link, base, tag, description_share)

    payload = _add(world, 'model', name='ropes_payload')
    _add(payload, 'pose', _text((*model.home, 0.0, 0.0, 0.0)))
    payload_link = _add(payload, 'link', name='end_effector')
    source_payload = robot.find("link[@name='end_effector']")
    inertial = _add(payload_link, 'inertial')
    _add(inertial, 'mass', settings.mass_kg)
    radius = float(source_payload.find('collision/geometry/sphere').get('radius'))
    inertia = _add(inertial, 'inertia')
    for axis in ('ixx', 'iyy', 'izz'):
        _add(inertia, axis, 0.4 * settings.mass_kg * radius ** 2)
    for tag in ('visual', 'collision'):
        _copy_geometry(robot, payload_link, source_payload, tag, description_share)

    plugin = _add(payload, 'plugin', name='cable_dynamics', filename='libropes_cable_plugin.so')
    _add(plugin, 'use_sim_time', str(use_sim_time).lower())
    _add(plugin, 'max_drum_speed_rad_s', settings.max_drum_speed_rad_s)
    _add(plugin, 'state_rate_hz', settings.state_rate_hz)
    _add(plugin, 'stiffness_n_m', settings.stiffness_n_m)
    _add(plugin, 'damping_ns_m', settings.damping_ns_m)
    _add(plugin, 'description_home', _text(model.home))
    static_tensions = model.tensions_per_mass(model.home, gravity_m_s2=settings.gravity_m_s2)
    for index, cable in enumerate(model.cables):
        source_joint = robot.find(f"joint[@name='{cable['joint']}']")
        child_name = source_joint.find('child').get('link')
        drum_name = f'ropes_{child_name}'
        drum = _add(world, 'model', name=drum_name)
        _add(drum, 'static', 'true')
        _add(drum, 'pose', _text(_urdf_pose(source_joint)))
        drum_link = _add(drum, 'link', name=child_name)
        source_drum = robot.find(f"link[@name='{child_name}']")
        _copy_geometry(robot, drum_link, source_drum, 'visual', description_share)

        span_name = f'cable_{index + 1}_span'
        for segment, (start, end) in enumerate(pairwise(cable['fixed_path_m'])):
            _cable_cylinder(frame_link, f'cable_{index + 1}_guide_{segment}',
                            start, end, 0.001, cable['color_rgb'])
        _cable_cylinder(frame_link, span_name, cable['exit_m'], model.home,
                        0.001, cable['color_rgb'])

        length = model.neutral_lengths[index]
        signed_radius = model.payout_coefficients[index]
        preload = static_tensions[index] * settings.mass_kg / settings.stiffness_n_m
        if not 0.0 < preload < length:
            raise ValueError(f'{cable["joint"]}: initial spring preload exceeds the nominal span')
        row = _add(plugin, 'cable')
        _add(row, 'joint', cable['joint'])
        _add(row, 'anchor', _text(cable['exit_m']))
        _add(row, 'neutral_length', length)
        _add(row, 'free_length_limit', model.free_length_limits[index])
        _add(row, 'payout_coefficient', signed_radius)
        _add(row, 'preload_m', preload)
        _add(row, 'drum_model', drum_name)
        _add(row, 'drum_axis', source_joint.find('axis').get('xyz'))
        _add(row, 'span_visual', f'ropes_frame::base_link::{span_name}')

    ET.indent(tree, space='  ')
    return tree
