"""Read joint arrays in model order at ROS boundaries."""

import math


def joint_positions(message, names):
    """Return finite positions for the required names in order."""
    names = tuple(names)
    if len(message.name) != len(message.position):
        raise ValueError("JointState name and position arrays must have the same length")
    if len(set(message.name)) != len(message.name):
        raise ValueError("JointState contains duplicate joint names")
    by_name = dict(zip(message.name, message.position))
    missing = [name for name in names if name not in by_name]
    if missing:
        raise ValueError(f"JointState is missing joints: {', '.join(missing)}")
    positions = tuple(by_name[name] for name in names)
    if any(not math.isfinite(position) for position in positions):
        raise ValueError("JointState positions for all required joints must be finite")
    return positions
