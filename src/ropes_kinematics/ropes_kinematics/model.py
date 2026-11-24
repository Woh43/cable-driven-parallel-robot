"""Fixed-guide, constant-radius kinematics for a gravity-supported point load."""

import json
import math
from numbers import Real
from pathlib import Path
from types import MappingProxyType


DEFAULT_GRAVITY_M_S2 = 9.81
LENGTH_TOLERANCE_M = 1.0e-9
MIN_GUIDE_PLANE_DISTANCE_M = 1.0e-6
MIN_TENSION_PER_MASS = 1.0e-8


def _number(value, label):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{label} must be a finite number")
    try:
        number = float(value)
    except OverflowError as error:
        raise ValueError(f"{label} must be a finite number") from error
    if not math.isfinite(number):
        raise ValueError(f"{label} must be a finite number")
    return number


def _vector(values, label, size=3):
    try:
        values = tuple(values)
    except TypeError as error:
        raise ValueError(f"{label} must contain {size} numbers") from error
    if len(values) != size:
        raise ValueError(f"{label} must contain {size} numbers")
    return tuple(_number(value, label) for value in values)


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _subtract(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _scale(vector, factor):
    return tuple(component * factor for component in vector)


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _read_cable(config):
    cable = dict(config)
    name = cable["joint"]
    if not isinstance(name, str) or not name.strip():
        raise ValueError("Each cable must have a joint name")

    cable["exit_m"] = _vector(cable["exit_m"], f"{name} exit_m")
    path = tuple(_vector(point, f"{name} fixed_path_m") for point in cable["fixed_path_m"])
    if len(path) < 2:
        raise ValueError(f"{name} fixed_path_m needs at least two points")
    if math.dist(path[-1], cable["exit_m"]) > MIN_GUIDE_PLANE_DISTANCE_M:
        raise ValueError(f"{name} fixed_path_m must end at exit_m")
    cable["fixed_path_m"] = path

    for field in (
        "radius_m", "payout_sign", "cut_length_m", "retained_wraps",
        "termination_allowance_m", "lead_length_m",
    ):
        cable[field] = _number(cable[field], f"{name} {field}")
    if cable["radius_m"] <= 0.0 or cable["cut_length_m"] <= 0.0:
        raise ValueError(f"{name} radius and cut length must be positive")
    if cable["payout_sign"] not in (-1, 1):
        raise ValueError(f"{name} payout_sign must be +1 or -1")
    if min(cable["retained_wraps"], cable["termination_allowance_m"],
           cable["lead_length_m"]) < 0.0:
        raise ValueError(
            f"{name} wraps, termination allowance and lead length must be nonnegative"
        )

    color = _vector(cable.get("color_rgb", (0.75, 0.75, 0.75)), f"{name} color_rgb")
    if min(color) < 0.0 or max(color) > 1.0:
        raise ValueError(f"{name} color_rgb components must be within [0, 1]")
    cable["color_rgb"] = color
    return MappingProxyType(cable)


class CableModel:
    """Solve XYZ position and unwrapped drum angles in the configured joint order."""

    @classmethod
    def from_file(cls, path):
        """Read the robot calibration from JSON."""
        path = Path(path).expanduser()
        try:
            config = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as error:
            raise ValueError(f"Cannot read cable configuration {path}: {error}") from error
        return cls(config)

    def __init__(self, config):
        try:
            self.home = _vector(config["home_m"], "home_m")
            self.height_range = _vector(config["height_range_m"], "height_range_m", size=2)
            self.cables = tuple(_read_cable(cable) for cable in config["cables"])
        except (KeyError, TypeError) as error:
            raise ValueError(f"Malformed cable geometry configuration: {error}") from error

        if self.height_range[0] >= self.height_range[1]:
            raise ValueError("height_range_m must have increasing limits")
        self.joint_names = tuple(cable["joint"] for cable in self.cables)
        if len(self.joint_names) != 3 or len(set(self.joint_names)) != 3:
            raise ValueError("Exactly three cables with distinct joint names are required")

        self.anchors = tuple(cable["exit_m"] for cable in self.cables)
        first, second, third = self.anchors
        ab = _subtract(second, first)
        ac = _subtract(third, first)
        horizontal_area = _cross(ab, ac)[2]
        if not math.isfinite(horizontal_area) or abs(horizontal_area) < 1.0e-12:
            raise ValueError("Cable exits must form a nonzero horizontal triangle")
        self._d = math.hypot(*ab)
        self._u = _scale(ab, 1.0 / self._d)
        self._i = _dot(self._u, ac)
        transverse = _subtract(ac, _scale(self._u, self._i))
        self._j = math.hypot(*transverse)
        if not math.isfinite(self._j) or self._j <= LENGTH_TOLERANCE_M:
            raise ValueError("Cable exits must define a nonsingular guide plane")
        self._v = _scale(transverse, 1.0 / self._j)
        normal = _cross(self._u, self._v)
        self._up = normal if normal[2] > 0.0 else _scale(normal, -1.0)

        self.neutral_lengths = tuple(math.dist(self.home, anchor) for anchor in self.anchors)
        self.payout_coefficients = tuple(
            cable["payout_sign"] * cable["radius_m"] for cable in self.cables
        )
        self.free_length_limits = tuple(
            cable["cut_length_m"] - cable["lead_length_m"] - cable["termination_allowance_m"]
            - math.tau * cable["radius_m"] * cable["retained_wraps"]
            for cable in self.cables
        )
        if any(not math.isfinite(limit) or limit <= 0.0 for limit in self.free_length_limits):
            raise ValueError(
                "Each cable must have a positive length budget after lead and wrap reserve"
            )
        try:
            calculated_home = self.forward((0.0, 0.0, 0.0))
            self.free_lengths(self.home)
        except ValueError as error:
            raise ValueError(f"Invalid home position: {error}") from error
        if math.dist(calculated_home, self.home) > 1.0e-8:
            raise ValueError("The home position does not match the lower hanging solution")

    def _validate_lengths(self, lengths):
        for name, length, limit in zip(self.joint_names, lengths, self.free_length_limits):
            if not math.isfinite(length) or length <= 0.0:
                raise ValueError(f"{name}: free cable length must be finite and positive")
            if length > limit + LENGTH_TOLERANCE_M:
                raise ValueError(
                    f"{name}: requested payout exceeds the cut length and wrap reserve"
                )

    def _validate_position(self, position):
        low, high = self.height_range
        if not low - LENGTH_TOLERANCE_M <= position[2] <= high + LENGTH_TOLERANCE_M:
            raise ValueError("End effector height is outside the configured range")
        plane_distance = _dot(_subtract(self.anchors[0], position), self._up)
        if not math.isfinite(plane_distance) or plane_distance <= MIN_GUIDE_PLANE_DISTANCE_M:
            raise ValueError(
                "End effector must lie on the nonsingular lower side of the guide plane"
            )

    def _point_lengths(self, position):
        self._validate_position(position)
        lengths = tuple(math.dist(position, anchor) for anchor in self.anchors)
        self._validate_lengths(lengths)
        return lengths

    def _tensions(self, position, lengths, support):
        first, second, third = (
            _scale(_subtract(anchor, position), 1.0 / length)
            for anchor, length in zip(self.anchors, lengths)
        )
        second_cross_third = _cross(second, third)
        first_cross_second = _cross(first, second)
        determinant = _dot(first, second_cross_third)
        if not math.isfinite(determinant) or abs(determinant) < 1.0e-12:
            raise ValueError("Cable directions are singular")
        # Each column of the force matrix is a unit direction from the load to a guide.
        tensions = (
            _dot(support, second_cross_third) / determinant,
            _dot(support, _cross(third, first)) / determinant,
            _dot(support, first_cross_second) / determinant,
        )
        if any(not math.isfinite(tension) or tension <= MIN_TENSION_PER_MASS
               for tension in tensions):
            raise ValueError(
                "The requested point or acceleration cannot keep all three cables taut"
            )
        return tensions

    def free_lengths(self, position):
        """Return the guide-to-effector lengths in meters at a feasible static position."""
        position = _vector(position, "end effector position")
        lengths = self._point_lengths(position)
        self._tensions(position, lengths, (0.0, 0.0, DEFAULT_GRAVITY_M_S2))
        return lengths

    def inverse(self, position):
        """Map a feasible base-frame XYZ target in meters to drum angles in radians."""
        angles = tuple(
            (length - neutral) / payout
            for length, neutral, payout in zip(
                self.free_lengths(position), self.neutral_lengths, self.payout_coefficients
            )
        )
        if not all(math.isfinite(angle) for angle in angles):
            raise ValueError("Requested position would require a nonfinite drum angle")
        return angles

    def cable_lengths(self, angles):
        """Convert unwrapped drum angles in radians to guide-to-effector lengths in meters."""
        angles = _vector(angles, "drum angles")
        lengths = tuple(
            neutral + payout * angle
            for neutral, payout, angle in zip(
                self.neutral_lengths, self.payout_coefficients, angles
            )
        )
        self._validate_lengths(lengths)
        return lengths

    def forward(self, angles):
        """Find the feasible lower sphere intersection from unwrapped drum angles."""
        return self.forward_from_lengths(self.cable_lengths(angles))

    def forward_from_lengths(self, lengths):
        """Find the feasible lower sphere intersection from three free lengths in meters."""
        lengths = _vector(lengths, "free cable lengths")
        self._validate_lengths(lengths)
        first, second, third = lengths
        x = ((first - second) * (first + second) + self._d * self._d) / (2.0 * self._d)
        y = (
            (first - third) * (first + third) + self._i * self._i + self._j * self._j
            - 2.0 * self._i * x
        ) / (2.0 * self._j)
        height_squared = first * first - x * x - y * y
        if not math.isfinite(height_squared) or height_squared <= MIN_GUIDE_PLANE_DISTANCE_M ** 2:
            raise ValueError("Free cable lengths have no nonsingular lower sphere intersection")
        below = math.sqrt(height_squared)
        position = tuple(
            anchor + x * u + y * v - below * up
            for anchor, u, v, up in zip(self.anchors[0], self._u, self._v, self._up)
        )
        self._validate_position(position)
        solved_lengths = tuple(math.dist(position, anchor) for anchor in self.anchors)
        if any(abs(expected - actual) > LENGTH_TOLERANCE_M
               for expected, actual in zip(lengths, solved_lengths)):
            raise ValueError("The reconstructed point does not match the given cable lengths")
        self._tensions(position, solved_lengths, (0.0, 0.0, DEFAULT_GRAVITY_M_S2))
        return position

    def jacobian(self, position):
        """Return d(drum angle)/d(XYZ) in radians per meter."""
        position = _vector(position, "end effector position")
        rows = tuple(
            tuple(component / length / payout for component in _subtract(position, anchor))
            for anchor, length, payout in zip(
                self.anchors, self.free_lengths(position), self.payout_coefficients
            )
        )
        if not all(math.isfinite(component) for row in rows for component in row):
            raise ValueError("The drum Jacobian is nonfinite")
        return rows

    def tensions_per_mass(self, position, acceleration=(0.0, 0.0, 0.0),
                          gravity_m_s2=DEFAULT_GRAVITY_M_S2):
        """Return required positive cable tensions as N/kg for a point acceleration."""
        position = _vector(position, "end effector position")
        lengths = self._point_lengths(position)
        ax, ay, az = _vector(acceleration, "end effector acceleration")
        gravity = _number(gravity_m_s2, "gravity magnitude")
        if gravity <= 0.0:
            raise ValueError("Gravity magnitude must be positive")
        return self._tensions(position, lengths, (ax, ay, az + gravity))

    def validate_acceleration(self, position, acceleration, gravity_m_s2=DEFAULT_GRAVITY_M_S2):
        """Raise ValueError if the required acceleration would slacken a cable."""
        self.tensions_per_mass(position, acceleration, gravity_m_s2)
