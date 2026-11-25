from collections import deque
from pathlib import Path
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

from geometry_msgs.msg import Point, PointStamped
import pytest
from rclpy.time import Time

from ropes_kinematics.model import CableModel
from ropes_motion.planner import CartesianPlanner
from ropes_motion.trajectory_server import (
    TrajectoryServer,
    _Execution,
    _Feedback,
    _MeasuredDrums,
    _MeasuredPose,
)


_NS = 1_000_000_000


@pytest.fixture
def model():
    config = Path(__file__).resolve().parents[2] / "ropes_description" / "config" / "cables.json"
    return CableModel.from_file(config)


@pytest.fixture
def settling_server():
    return SimpleNamespace(
        goal_tolerance_m=0.005,
        actuator_goal_tolerance_rad=0.02,
        settle_dwell_s=0.15,
        settle_timeout_s=0.5,
        _finish=Mock(),
    )


def _measure(model, position, source_ns, received_ns=None):
    if received_ns is None:
        received_ns = source_ns
    point = Point(x=position[0], y=position[1], z=position[2])
    pose = _MeasuredPose(PointStamped(point=point), received_ns, source_ns)
    drums = _MeasuredDrums(model.inverse(position), received_ns, source_ns)
    return _Feedback(pose, drums)


def test_hardware_goal_waits_for_both_cartesian_and_drum_agreement(model, settling_server):
    target = (0.0, 0.0, 0.150)
    plan = CartesianPlanner(model).plan(model.home, [target])
    execution = _Execution(None, None, plan=plan)
    angles = model.inverse(target)
    waypoint, = plan.waypoint_schedule()
    actual = (0.0, 0.0, 0.154)
    for now_ns in (_NS, 1_170_000_000, 1_350_000_000, 1_500_000_000):
        feedback = _measure(model, actual, now_ns)
        TrajectoryServer._settle(
            settling_server, execution, now_ns, feedback, waypoint, angles,
        )
    args = settling_server._finish.call_args.args
    assert args[0] is execution and args[1] == "abort"
    assert "Cartesian error 0.0040 m" in args[2] and "drum error 0.1707 rad" in args[2]


def test_hardware_waypoint_waits_for_fresh_samples_and_rebases_plan_time(model, settling_server):
    first = (0.0, 0.0, 0.150)
    last = (0.0, 0.0, 0.170)
    plan = CartesianPlanner(model).plan(model.home, [first, last])
    schedule = plan.waypoint_schedule()
    execution = _Execution(None, None, plan=plan, hardware_waypoints=schedule)
    angles = model.inverse(first)

    def observe(now_ns, source_ns):
        feedback = _measure(model, first, source_ns, now_ns)
        TrajectoryServer._settle(
            settling_server, execution, now_ns, feedback, schedule[0], angles,
        )

    observe(_NS, _NS)
    observe(1_070_000_000, 990_000_000)
    assert execution.settled_since_ns is None  # A delayed pre-command sample cannot settle it.
    observe(1_130_000_000, 1_120_000_000)
    observe(1_290_000_000, 1_120_000_000)
    assert execution.waypoint_cursor == 0  # Repeated sample timestamp adds no dwell.
    observe(1_320_000_000, 1_310_000_000)
    assert execution.waypoint_cursor == 1
    assert execution.waypoint_command_ns is None and execution.settled_since_ns is None
    assert execution.started_ns == 1_320_000_000 - round(schedule[0].elapsed_s * _NS)
    settling_server._finish.assert_not_called()


def test_hardware_feedback_uses_latest_fresh_common_timestamp(model):
    common = _measure(model, model.home, _NS)
    newer = _measure(model, (0.0, 0.0, 0.169), _NS + 10_000_000)
    server = SimpleNamespace(
        require_hardware_ready=True, pose_timeout_s=0.1,
        _pose_samples=deque([common.pose, newer.pose]),
        _drum_samples=deque([common.drums]),
    )

    def fresh(sample, stamp):
        return TrajectoryServer._fresh_sample(server, sample, stamp)

    server._fresh_sample = fresh
    assert TrajectoryServer._fresh_feedback(server, _NS + 20_000_000) == common
    server._drum_samples.append(newer.drums)
    assert TrajectoryServer._fresh_feedback(server, _NS + 20_000_000) == newer
    assert TrajectoryServer._fresh_feedback(server, _NS + 120_000_000) is None


@pytest.mark.parametrize("hardware", [False, True])
def test_drum_command_velocity_matches_the_position_trajectory(model, hardware):
    plan = CartesianPlanner(model).plan(model.home, [(0.0, 0.0, 0.15)])
    handle = SimpleNamespace(is_cancel_requested=False, publish_feedback=Mock())
    execution = _Execution(handle, None, plan=plan, started_ns=_NS)
    if hardware:
        execution.hardware_waypoints = plan.waypoint_schedule()
    clock = SimpleNamespace(now=Mock(return_value=Time(nanoseconds=1_200_000_000)))
    server = SimpleNamespace(
        model=model, require_hardware_ready=hardware, pose_timeout_s=0.5, command_period_s=0.02,
        _execution=execution, get_clock=Mock(return_value=clock), _hardware_problem=Mock(),
        _fresh_feedback=Mock(return_value=_measure(model, model.home, _NS)), _commands=Mock(),
        _settle=Mock(), _finish=Mock(),
    )
    server._hardware_problem.return_value = None
    server._command = MethodType(TrajectoryServer._command, server)
    TrajectoryServer._advance(server)
    message, = server._commands.publish.call_args.args
    assert list(message.name) == list(model.joint_names)
    assert list(message.position) == pytest.approx(model.inverse(plan.sample(0.2).position))

    if hardware:
        assert list(message.velocity) == []
        return

    step = 1e-5
    before = model.inverse(plan.sample(0.2 - step).position)
    after = model.inverse(plan.sample(0.2 + step).position)
    finite_difference = tuple((a - b) / (2.0 * step) for a, b in zip(after, before))
    assert list(message.velocity) == pytest.approx(finite_difference)
    assert message.velocity[0] > 0.0 and message.velocity[2] < 0.0

    end_ns = _NS + round(plan.duration_s * _NS) + 1
    execution.last_tick_ns = end_ns - 20_000_000
    clock.now.return_value = Time(nanoseconds=end_ns)
    TrajectoryServer._advance(server)
    message, = server._commands.publish.call_args.args
    assert list(message.velocity) == [0.0, 0.0, 0.0]

    handle.is_cancel_requested = True
    TrajectoryServer._advance(server)
    message, = server._commands.publish.call_args.args
    assert list(message.velocity) == [0.0, 0.0, 0.0]
