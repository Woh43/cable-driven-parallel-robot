"""Numerical checks against the CAD calibration and its feasible workspace."""

import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from ropes_kinematics.joints import joint_positions
from ropes_kinematics.model import CableModel


CONFIG_PATH = (
    Path(__file__).resolve().parents[2] / "ropes_description" / "config" / "cables.json"
)


@pytest.fixture
def model():
    return CableModel.from_file(CONFIG_PATH)


def test_home_and_vertical_offset(model):
    assert model.home == (0.0, 0.0, 0.17)
    assert model.inverse(model.home) == (0.0, 0.0, 0.0)
    assert model.forward((0.0, 0.0, 0.0)) == pytest.approx(model.home, abs=1.0e-12)
    assert model.inverse((0.0, 0.0, 0.15)) == pytest.approx(
        (0.661, 0.845, -0.661), abs=0.0006
    )


@pytest.mark.parametrize("height", [0.09, 0.12, 0.17, 0.20, 0.24])
def test_translational_workspace_round_trip(model, height):
    for x in (-0.035, -0.02, 0.0, 0.03, 0.035):
        for y in (-0.035, -0.01, 0.0, 0.02, 0.035):
            position = (x, y, height)
            angles = model.inverse(position)
            lengths = model.cable_lengths(angles)
            assert lengths == pytest.approx(model.free_lengths(position), abs=1.0e-12)
            assert model.forward_from_lengths(lengths) == pytest.approx(position, abs=1.0e-12)
            assert model.forward(angles) == pytest.approx(position, abs=1.0e-12)


def test_jacobian_matches_finite_differences(model):
    position = (0.012, -0.031, 0.20)
    jacobian = model.jacobian(position)
    step = 1.0e-6
    for axis in range(3):
        plus = list(position)
        minus = list(position)
        plus[axis] += step
        minus[axis] -= step
        expected = tuple(
            (high - low) / (2.0 * step)
            for high, low in zip(model.inverse(plus), model.inverse(minus))
        )
        assert tuple(row[axis] for row in jacobian) == pytest.approx(expected, abs=1.0e-7)


def test_tensions_reproduce_gravity_and_commanded_acceleration(model):
    acceleration = (0.10, -0.03, 0.40)
    position = (0.01, -0.02, 0.15)
    tensions = model.tensions_per_mass(position, acceleration)
    lengths = model.free_lengths(position)
    support = tuple(
        sum(tension * (anchor[axis] - position[axis]) / length
            for tension, anchor, length in zip(tensions, model.anchors, lengths))
        for axis in range(3)
    )
    assert min(tensions) > 0.0
    assert support == pytest.approx((0.10, -0.03, 9.81 + 0.40), abs=1.0e-12)
    with pytest.raises(ValueError, match="taut"):
        model.validate_acceleration(position, (30.0, 0.0, 0.0))


def test_rejects_unreachable_positions_and_lengths(model):
    with pytest.raises(ValueError, match="height"):
        model.inverse((0.0, 0.0, 0.25))
    with pytest.raises(ValueError, match="taut"):
        model.inverse((0.0, 0.15, 0.17))
    with pytest.raises(ValueError, match="wrap reserve"):
        model.forward((100.0, 0.0, 0.0))
    with pytest.raises(ValueError, match="positive"):
        model.forward((-100.0, 0.0, 0.0))
    with pytest.raises(ValueError, match="sphere intersection"):
        model.forward_from_lengths((0.1, 0.1, 0.1))


def test_rejects_the_reflected_solution_when_both_heights_are_allowed():
    config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    config["height_range_m"] = [0.09, 0.65]
    model = CableModel(config)
    first, second, third = model.anchors
    slope = (second[2] - first[2]) / (second[1] - first[1])
    plane_at_home_y = first[2] - slope * first[1]
    distance = (plane_at_home_y - model.home[2]) / math.hypot(1.0, slope)
    mirror = (0.0, -2.0 * distance * slope / math.hypot(1.0, slope),
              model.home[2] + 2.0 * distance / math.hypot(1.0, slope))
    neutral_lengths = tuple(math.dist(mirror, anchor) for anchor in (first, second, third))
    assert neutral_lengths == pytest.approx(model.neutral_lengths, abs=1.0e-12)
    with pytest.raises(ValueError, match="lower side"):
        model.inverse(mirror)
    assert model.forward_from_lengths(neutral_lengths) == pytest.approx(model.home)


def test_rejects_invalid_configuration_and_nonfinite_commands(model):
    config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    config["cables"][1]["joint"] = config["cables"][0]["joint"]
    with pytest.raises(ValueError, match="distinct"):
        CableModel(config)
    for position in ((0.0, math.nan, 0.17), (math.inf, 0.0, 0.17),
                     (False, 0.0, 0.17), (10**400, 0.0, 0.17)):
        with pytest.raises(ValueError, match="finite"):
            model.inverse(position)
    with pytest.raises(ValueError, match="3 numbers"):
        model.forward((0.0, 0.0))


def test_joint_message_order_and_validation():
    names = ("drum_1_joint", "drum_2_joint", "drum_3_joint")
    message = SimpleNamespace(
        name=["unrelated", names[2], names[0], names[1]],
        position=[100.0, 3.0, 1.0, 2.0],
    )
    assert joint_positions(message, names) == (1.0, 2.0, 3.0)
    with pytest.raises(ValueError, match="missing"):
        joint_positions(SimpleNamespace(name=[names[0]], position=[0.0]), names)
    with pytest.raises(ValueError, match="duplicate"):
        joint_positions(SimpleNamespace(name=[names[0], names[0]], position=[0.0, 0.0]), names)
