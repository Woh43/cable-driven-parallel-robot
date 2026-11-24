"""Load the CAD model for live state and extract the selected slider joints."""

from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory


BASE_FRAME = 'base_link'
CARTESIAN_JOINTS = ('end_effector_x_joint', 'end_effector_y_joint', 'end_effector_z_joint')
DESCRIPTION_SHARE = Path(get_package_share_directory('ropes_description'))
DEFAULT_GEOMETRY_PATH = DESCRIPTION_SHARE / 'config' / 'cables.json'
ROBOT_URDF_PATH = DESCRIPTION_SHARE / 'urdf' / 'cable_robot.urdf'
XY_SLIDER_LIMIT_M = 0.035


def _live_robot(model):
    robot = ET.parse(ROBOT_URDF_PATH).getroot()
    previews = [
        link for link in robot.findall('link') if link.get('name').startswith('cable_preview_')
    ]
    preview_names = {link.get('name') for link in previews}
    for joint in robot.findall('joint'):
        if joint.find('child').get('link') in preview_names:
            robot.remove(joint)
    for link in previews:
        robot.remove(link)

    joints = {joint.get('name'): joint for joint in robot.findall('joint')}
    unknown_joints = set(model.joint_names).difference(joints)
    if unknown_joints:
        raise ValueError(f'Calibrated joints are missing from the URDF: {sorted(unknown_joints)}')
    home_origin = joints[CARTESIAN_JOINTS[0]].find('origin')
    home_origin.set('xyz', ' '.join(map(str, model.home)))
    for index, cable in enumerate(model.cables, start=1):
        exit_origin = joints[f'cable_exit_{index}_joint'].find('origin')
        exit_origin.set('xyz', ' '.join(map(str, cable['exit_m'])))

    z_limit = joints[CARTESIAN_JOINTS[2]].find('limit')
    z_limit.set('lower', str(model.height_range[0] - model.home[2]))
    z_limit.set('upper', str(model.height_range[1] - model.home[2]))
    return robot


def robot_description(model):
    return ET.tostring(_live_robot(model), encoding='unicode')


def slider_description(model, mode):
    joint_names = CARTESIAN_JOINTS if mode == 'xyz' else model.joint_names
    joints = {joint.get('name'): joint for joint in _live_robot(model).findall('joint')}
    selected = [joints[name] for name in joint_names]
    link_names = dict.fromkeys(
        joint.find(tag).get('link') for joint in selected for tag in ('parent', 'child')
    )
    controls = ET.Element('robot', name='ropes_slider_controls')
    for name in link_names:
        ET.SubElement(controls, 'link', name=name)
    for joint in selected:
        if joint.get('name') in CARTESIAN_JOINTS[:2]:
            limits = joint.find('limit')
            limits.set('lower', str(max(float(limits.get('lower')), -XY_SLIDER_LIMIT_M)))
            limits.set('upper', str(min(float(limits.get('upper')), XY_SLIDER_LIMIT_M)))
        controls.append(joint)
    return ET.tostring(controls, encoding='unicode')
