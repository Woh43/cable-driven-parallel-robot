import math
from pathlib import Path

import pytest

from ropes_kinematics.model import CableModel
from ropes_motion.planner import ActuatorAngleLimits, CartesianPlanner, TrajectoryLimits


@pytest.fixture
def model():
    description = Path(__file__).resolve().parents[2] / "ropes_description"
    return CableModel.from_file(description / "config" / "cables.json")


def test_waypoint_endpoints_are_at_rest(model):
    planner = CartesianPlanner(model)
    first = (0.025, -0.015, 0.20)
    last = (-0.012, 0.030, 0.15)
    first_duration = planner.plan(model.home, [first]).duration_s
    plan = planner.plan(model.home, [first, last])

    for time_s, expected in ((0.0, model.home), (first_duration, first), (plan.duration_s, last)):
        sample = plan.sample(time_s)
        assert sample.position == expected
        assert sample.velocity == sample.acceleration == (0.0, 0.0, 0.0)
    assert plan.sample(-1.0).position == model.home
    assert plan.sample(plan.duration_s + 1.0).position == last
    assert plan.sample(first_duration / 2.0).position == pytest.approx(
        tuple((start + end) / 2.0 for start, end in zip(model.home, first))
    )


def test_limits_and_continuous_cable_support(model):
    limits = TrajectoryLimits(
        max_cartesian_speed_m_s=0.20, max_cartesian_acceleration_m_s2=1.0,
        max_drum_speed_rad_s=0.8,
    )
    plan = CartesianPlanner(model, limits).plan(
        model.home, [(0.030, 0.018, 0.22), (-0.026, -0.02, 0.11)]
    )
    speeds = []
    for index in range(501):
        point = plan.sample(index * plan.duration_s / 500.0)
        assert math.hypot(*point.velocity) <= limits.max_cartesian_speed_m_s + 1e-9
        assert math.hypot(*point.acceleration) <= limits.max_cartesian_acceleration_m_s2 + 1e-9
        model.validate_acceleration(point.position, point.acceleration)
        for row in model.jacobian(point.position):
            speed = abs(sum(jac * vel for jac, vel in zip(row, point.velocity)))
            assert speed <= limits.max_drum_speed_rad_s + 1e-9
            speeds.append(speed)
    assert max(speeds) > 0.7  # A drum speed bound is active on the longer segment.


def test_planner_slows_near_taut_workspace_edge(model):
    start = (0.0, -0.12, 0.17)
    end = (0.012, -0.162, 0.17)
    limits = TrajectoryLimits(max_cartesian_speed_m_s=1.0, max_drum_speed_rad_s=50.0)
    delta = tuple(b - a for a, b in zip(start, end))
    distance = math.hypot(*delta)
    acceleration = tuple(d * limits.max_cartesian_acceleration_m_s2 / distance for d in delta)
    with pytest.raises(ValueError, match="taut"):
        model.validate_acceleration(end, acceleration)
    plan = CartesianPlanner(model, limits).plan(start, [end])
    acceleration_peak = plan.sample(plan.duration_s * (3.0 - math.sqrt(3.0)) / 6.0)
    assert math.hypot(*acceleration_peak.acceleration) < 0.09
    for index in range(101):
        point = plan.sample(index * plan.duration_s / 100.0)
        model.validate_acceleration(point.position, point.acceleration)


def test_duplicate_waypoints_and_stationary_path(model):
    planner = CartesianPlanner(model)
    end = (0.020, 0.020, 0.17)
    plan = planner.plan(model.home, [model.home, end, end])
    assert plan.points == (model.home, end)
    assert plan.sample(plan.duration_s / 2.0).segment_index == 1
    assert plan.sample(plan.duration_s).segment_index == 2
    assert plan.waypoint_schedule()[0].elapsed_s == plan.duration_s
    assert plan.waypoint_schedule()[0].segment_index == 2
    stationary = planner.plan(model.home, [model.home, model.home])
    assert stationary.duration_s == 0.0
    assert stationary.sample(10.0).position == model.home
    assert stationary.waypoint_schedule()[0].position == model.home


def test_invalid_waypoints_are_reported_before_execution(model):
    planner = CartesianPlanner(model)
    with pytest.raises(ValueError, match="Waypoint 2 is unreachable"):
        planner.plan(model.home, [(0.020, 0.020, 0.17), (0.0, 0.0, 1.0)])
    with pytest.raises(ValueError, match="Current position"):
        planner.plan((0.0, 0.0, 1.0), [model.home])
    with pytest.raises(ValueError, match="finite"):
        planner.plan(model.home, [(0.0, 0.0, math.nan)])
    with pytest.raises(ValueError, match="At least one"):
        planner.plan(model.home, [])


def test_calibrated_drum_limits_check_start_and_later_segments(model):
    unbounded = math.inf
    limits = ActuatorAngleLimits(
        lower_rad=(-unbounded, -unbounded, -unbounded),
        upper_rad=(0.5, unbounded, unbounded),
    )
    planner = CartesianPlanner(model, actuator_limits=limits)

    out_of_range = (0.0, 0.0, 0.15)
    with pytest.raises(ValueError, match="Current position.*drum_1_joint"):
        planner.plan(out_of_range, [out_of_range])
    with pytest.raises(ValueError, match="Waypoint 2.*drum_1_joint.*calibrated drum range"):
        planner.plan(model.home, [(0.0, 0.0, 0.16), out_of_range])
    assert planner.plan(model.home, [(0.0, 0.0, 0.16)]).duration_s > 0.0


@pytest.mark.parametrize("joint_index", [1, 2])
def test_calibrated_angle_extremum_inside_straight_segment(model, joint_index):
    if joint_index == 1:
        start = (-0.035, 0.0, 0.17)
        end = (0.035, 0.0, 0.17)
        lower, upper = (0.05, math.inf)
    else:
        start = (-0.03, 0.03, 0.17)
        end = (0.03, -0.03, 0.17)
        lower, upper = (-math.inf, -0.05)
    for point in (start, end):
        assert lower < model.inverse(point)[joint_index] < upper
    assert not lower < model.inverse(model.home)[joint_index] < upper

    lowers = [-math.inf] * 3
    uppers = [math.inf] * 3
    lowers[joint_index] = lower
    uppers[joint_index] = upper
    planner = CartesianPlanner(model, actuator_limits=ActuatorAngleLimits(lowers, uppers))
    with pytest.raises(ValueError, match=f"Waypoint 1.*{model.joint_names[joint_index]}"):
        planner.plan(start, [end])


def test_calibrated_bounds_are_inclusive(model):
    start = (0.0, 0.0, 0.16)
    end = (0.0, 0.0, 0.15)
    endpoints = model.inverse(start), model.inverse(end)
    limits = ActuatorAngleLimits(
        tuple(map(min, zip(*endpoints))), tuple(map(max, zip(*endpoints)))
    )
    planner = CartesianPlanner(model, actuator_limits=limits)
    assert planner.plan(start, [end]).sample(100.0).position == end
    assert planner.plan(end, [end]).duration_s == 0.0


@pytest.mark.parametrize("lower, upper", [
    ((-1.0, -1.0), (1.0, 1.0, 1.0)),
    ((-1.0, -1.0, 2.0), (1.0, 1.0, 1.0)),
    ((-1.0, -1.0, math.nan), (1.0, 1.0, 1.0)),
    ((-1.0, -1.0, math.inf), (1.0, 1.0, 1.0)),
    ((-1.0, -1.0, -1.0), (1.0, 1.0, -math.inf)),
])
def test_calibrated_bounds_reject_invalid_arrays(lower, upper):
    with pytest.raises(ValueError):
        ActuatorAngleLimits(lower, upper)


@pytest.mark.parametrize("value", [False, 0.0, -0.1, math.inf, math.nan])
def test_limits_require_positive_finite_values(value):
    with pytest.raises(ValueError, match="max_drum_speed_rad_s"):
        TrajectoryLimits(max_drum_speed_rad_s=value)
