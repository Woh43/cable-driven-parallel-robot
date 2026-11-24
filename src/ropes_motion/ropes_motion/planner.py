"""ROS-independent Cartesian waypoint planning for a three-cable point load."""

from bisect import bisect_right
from dataclasses import dataclass
import math
from numbers import Real

from ropes_kinematics.model import DEFAULT_GRAVITY_M_S2, MIN_TENSION_PER_MASS


_QUINTIC_PEAK_SPEED = 15.0 / 8.0
_QUINTIC_PEAK_ACCELERATION = 10.0 / math.sqrt(3.0)

Vector = tuple[float, float, float]
_ZERO: Vector = (0.0, 0.0, 0.0)


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite number")
    try:
        value = float(value)
    except OverflowError as error:
        raise ValueError(f"{name} must be a finite number") from error
    if not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return value


def _position(values, name):
    try:
        x, y, z = values
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must contain three Cartesian coordinates") from error
    return (_finite(x, name), _finite(y, name), _finite(z, name))


def _subtract(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _scale(vector, scale):
    return tuple(value * scale for value in vector)


def _angle_bounds(values, name, open_bound):
    try:
        values = tuple(values)
    except TypeError as error:
        raise ValueError(f"{name} must contain three joint bounds") from error
    if len(values) != 3:
        raise ValueError(f"{name} must contain three joint bounds")
    return tuple(open_bound if value == open_bound else _finite(value, name) for value in values)


@dataclass(frozen=True)
class ActuatorAngleLimits:
    """Inclusive drum travel limits in the model's joint order."""

    lower_rad: Vector = (-math.inf, -math.inf, -math.inf)
    upper_rad: Vector = (math.inf, math.inf, math.inf)

    def __post_init__(self):
        lower = _angle_bounds(self.lower_rad, "actuator_lower_limits_rad", -math.inf)
        upper = _angle_bounds(self.upper_rad, "actuator_upper_limits_rad", math.inf)
        if any(lo > hi for lo, hi in zip(lower, upper)):
            raise ValueError("Each actuator lower angle limit must be at or below its upper limit")
        object.__setattr__(self, "lower_rad", lower)
        object.__setattr__(self, "upper_rad", upper)

    def validate(self, joint_names, ranges):
        for name, bounds, lower, upper in zip(joint_names, ranges, self.lower_rad, self.upper_rad):
            minimum, maximum = bounds
            if minimum < lower or maximum > upper:
                raise ValueError(
                    f"{name}: path needs [{minimum:.6g}, {maximum:.6g}] rad outside "
                    f"the calibrated drum range [{lower:.6g}, {upper:.6g}] rad"
                )


@dataclass(frozen=True)
class TrajectoryLimits:
    max_cartesian_speed_m_s: float = 0.06
    max_cartesian_acceleration_m_s2: float = 0.18
    max_drum_speed_rad_s: float = 4.0
    min_segment_duration_s: float = 0.10
    gravity_m_s2: float = DEFAULT_GRAVITY_M_S2

    def __post_init__(self):
        for name, value in vars(self).items():
            if _finite(value, name) <= 0.0:
                raise ValueError(f"{name} must be positive")


@dataclass(frozen=True)
class TrajectorySample:
    position: Vector
    velocity: Vector
    acceleration: Vector
    segment_index: int


@dataclass(frozen=True)
class ScheduledWaypoint:
    elapsed_s: float
    position: Vector
    segment_index: int


@dataclass(frozen=True)
class _Segment:
    start: Vector
    end: Vector
    delta: Vector
    duration_s: float
    index: int

    def sample(self, elapsed_s):
        if elapsed_s <= 0.0:
            return TrajectorySample(self.start, _ZERO, _ZERO, self.index)
        if elapsed_s >= self.duration_s:
            return TrajectorySample(self.end, _ZERO, _ZERO, self.index)

        u = elapsed_s / self.duration_s
        # Use the symmetric form near the terminal point to avoid cancellation.
        edge = min(u, 1.0 - u)
        s = edge ** 3 * (10.0 + edge * (-15.0 + 6.0 * edge))
        if u <= 0.5:
            position = tuple(p + s * d for p, d in zip(self.start, self.delta))
        else:
            position = tuple(p - s * d for p, d in zip(self.end, self.delta))
        rate = 30.0 * (u * (1.0 - u)) ** 2 / self.duration_s
        acceleration = 60.0 * u * (1.0 - u) * (1.0 - 2.0 * u) / self.duration_s / self.duration_s
        return TrajectorySample(
            position, _scale(self.delta, rate), _scale(self.delta, acceleration), self.index
        )


@dataclass(frozen=True)
class CartesianPlan:
    points: tuple[Vector, ...]
    _segments: tuple[_Segment, ...]
    _end_times_s: tuple[float, ...]
    _terminal_index: int

    @property
    def duration_s(self):
        return self._end_times_s[-1] if self._end_times_s else 0.0

    def waypoint_schedule(self):
        """Return each distinct arrival time and its waypoint in execution order."""
        arrivals = tuple(
            ScheduledWaypoint(end_s, segment.end, segment.index)
            for end_s, segment in zip(self._end_times_s, self._segments)
        )
        terminal = ScheduledWaypoint(self.duration_s, self.points[-1], self._terminal_index)
        return (*arrivals[:-1], terminal)

    def sample(self, elapsed_s):
        """Return position and derivatives in SI units, clamped to the plan endpoints."""
        elapsed_s = _finite(elapsed_s, "elapsed time")
        if elapsed_s >= self.duration_s or not self._segments:
            return TrajectorySample(self.points[-1], _ZERO, _ZERO, self._terminal_index)
        index = bisect_right(self._end_times_s, elapsed_s)
        start_s = self._end_times_s[index - 1] if index else 0.0
        return self._segments[index].sample(elapsed_s - start_s)


class CartesianPlanner:
    """Time straight Cartesian segments with zero velocity and acceleration at waypoints."""

    def __init__(self, model, limits=None, actuator_limits=None):
        self.model = model
        self.limits = limits if limits is not None else TrajectoryLimits()
        self.actuator_limits = (
            actuator_limits if actuator_limits is not None else ActuatorAngleLimits()
        )

    def plan(self, start, waypoints):
        start = _position(start, "Current position")
        try:
            angles = self.model.inverse(start)
            point_ranges = ((angle, angle) for angle in angles)
            self.actuator_limits.validate(self.model.joint_names, point_ranges)
        except ValueError as error:
            raise ValueError(f"Current position cannot start a path: {error}") from error

        points = [start]
        segments = []
        end_times = []
        elapsed_s = 0.0
        count = 0
        for index, waypoint in enumerate(waypoints):
            count += 1
            end = _position(waypoint, f"Waypoint {index + 1}")
            if end == points[-1]:
                continue
            try:
                segment = self._make_segment(points[-1], end, index)
            except ValueError as error:
                raise ValueError(f"Waypoint {index + 1} is unreachable: {error}") from error
            end_s = elapsed_s + segment.duration_s
            if not math.isfinite(end_s) or end_s <= elapsed_s:
                raise ValueError("Path duration is outside numerical range")
            elapsed_s = end_s
            points.append(end)
            segments.append(segment)
            end_times.append(elapsed_s)

        if count == 0:
            raise ValueError("At least one waypoint is required")
        return CartesianPlan(tuple(points), tuple(segments), tuple(end_times), count - 1)

    def _make_segment(self, start, end, index):
        delta = _subtract(end, start)
        distance = math.hypot(*delta)
        if distance == 0.0 or not math.isfinite(distance):
            raise ValueError("Waypoint displacement is outside numerical range")
        direction = tuple(component / distance for component in delta)
        endpoints = (start, end)
        # Each free length is convex on a line. Its derivative along the line
        # is nondecreasing, so the maximum absolute drum derivative is at an end.
        drum_derivative = max(
            abs(_dot(row, delta))
            for point in endpoints
            for row in self.model.jacobian(point)
        )
        endpoint_lengths = tuple(
            tuple(math.dist(point, anchor) for anchor in self.model.anchors) for point in endpoints
        )
        minimum_lengths = self._minimum_lengths(start, direction, distance, endpoint_lengths)
        self._check_angle_limits(endpoint_lengths, minimum_lengths)
        acceleration = self._taut_acceleration_limit(
            endpoints, direction, endpoint_lengths, minimum_lengths
        )
        duration_s = max(
            self.limits.min_segment_duration_s,
            _QUINTIC_PEAK_SPEED * distance / self.limits.max_cartesian_speed_m_s,
            math.sqrt(_QUINTIC_PEAK_ACCELERATION * distance / acceleration),
            _QUINTIC_PEAK_SPEED * drum_derivative / self.limits.max_drum_speed_rad_s,
        )
        if not math.isfinite(duration_s):
            raise ValueError("Segment duration is nonfinite")
        return _Segment(start, end, delta, duration_s, index)

    def _minimum_lengths(self, start, direction, distance, endpoint_lengths):
        minimums = []
        for anchor, first, last in zip(self.model.anchors, *endpoint_lengths):
            along = _dot(_subtract(anchor, start), direction)
            if along <= 0.0:
                minimum = first
            elif along >= distance:
                minimum = last
            else:
                closest = tuple(p + along * axis for p, axis in zip(start, direction))
                minimum = math.dist(closest, anchor)
            minimums.append(minimum)
        return tuple(minimums)

    def _check_angle_limits(self, endpoint_lengths, minimum_lengths):
        ranges = []
        for first, last, minimum, neutral, payout in zip(
            *endpoint_lengths, minimum_lengths, self.model.neutral_lengths,
            self.model.payout_coefficients,
        ):
            # The projection gives minimum free length, and the maximum is
            # at an endpoint. Negative payout reverses the angle extrema.
            closest_angle = (minimum - neutral) / payout
            farthest_angle = (max(first, last) - neutral) / payout
            ranges.append((
                min(closest_angle, farthest_angle), max(closest_angle, farthest_angle)
            ))
        self.actuator_limits.validate(self.model.joint_names, ranges)

    def _taut_acceleration_limit(self, endpoints, direction, endpoint_lengths, minimum_lengths):
        def maintains_tension(acceleration, gravity):
            # For a fixed force, tension divided by free length is affine along
            # the segment divided by its positive distance to the guide plane.
            # Its extrema are therefore at the ends. For a fixed point it is
            # affine in acceleration, so the four corners bound it everywhere.
            for point, lengths in zip(endpoints, endpoint_lengths):
                for sign in (-1.0, 1.0):
                    try:
                        tensions = self.model.tensions_per_mass(
                            point, _scale(direction, sign * acceleration), gravity
                        )
                    except ValueError:
                        return False
                    for tension, minimum, length in zip(tensions, minimum_lengths, lengths):
                        if tension * (minimum / length) <= MIN_TENSION_PER_MASS:
                            return False
            return True

        gravity = self.limits.gravity_m_s2
        if (gravity != DEFAULT_GRAVITY_M_S2 and
                not maintains_tension(0.0, DEFAULT_GRAVITY_M_S2)):
            raise ValueError("The segment has insufficient static cable tension")
        upper = self.limits.max_cartesian_acceleration_m_s2
        if maintains_tension(upper, gravity):
            return upper
        if not maintains_tension(0.0, gravity):
            raise ValueError("The segment has insufficient static cable tension")

        # Retain the feasible side of the bracket to slow motions near an edge.
        lower = 0.0
        for _ in range(32):
            candidate = lower + (upper - lower) / 2.0
            if maintains_tension(candidate, gravity):
                lower = candidate
            else:
                upper = candidate
        if lower == 0.0:
            raise ValueError("The segment cannot maintain tension during acceleration")
        return lower
