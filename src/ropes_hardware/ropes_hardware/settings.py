"""Read hardware calibration and convert between drum and motor coordinates."""

from dataclasses import dataclass
import math
from numbers import Real
from pathlib import Path
import posixpath
import re

import yaml
from yaml.constructor import ConstructorError


PROTOCOL = "emm_v5_uart_0x6b"
SUPPORTED_BAUDRATES = (9600, 19200, 25000, 38400, 57600, 115200, 256000, 512000, 921600)
BLUETOOTH_UART_BAUDRATE = 115200
LEGACY_MAX_MOTOR_RPM = 5000
_RAD_S_PER_RPM = math.tau / 60.0
_MAX_PULSES_PER_REVOLUTION = 102400

_AXIS_FIELDS = {
    "address", "pulses_per_revolution", "direction",
    "gear_ratio", "lower_rad", "upper_rad",
}
_BLUETOOTH_TIMEOUTS = {
    "bluetooth_connect_timeout_s": 15.0,
    "bluetooth_response_timeout_s": 0.40,
}
_BLUETOOTH_DEFAULTS = {
    "poll_rate_hz": 5.0,
    "command_rate_hz": 5.0,
    "command_timeout_s": 0.75,
    "max_drum_speed_rad_s": 0.20,
}
_BLUETOOTH_FIELDS = {"bluetooth_address", *_BLUETOOTH_TIMEOUTS}
_BLUETOOTH_MAC = re.compile(r"(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}")
_DEFAULTS = {
    "allow_motion": False,
    "baudrate": 115200,
    "timeout_s": 0.05,
    "poll_rate_hz": 15.0,
    "command_rate_hz": 12.0,
    "command_timeout_s": 0.40,
    "max_drum_speed_rad_s": 0.30,
    "max_command_lead_rad": 0.06,
    "hold_tolerance_rad": 0.02,
    "acceleration": 200,
}

Angles = tuple[float, float, float]


class _UniqueKeysLoader(yaml.SafeLoader):
    """Reject duplicate YAML keys so motion settings cannot be hidden by overrides."""

    def construct_mapping(self, node, deep=False):
        self.flatten_mapping(node)
        mapping = {}
        for key_node, value_node in node.value:
            key = self.construct_object(key_node, deep=deep)
            if not isinstance(key, str):
                raise ConstructorError(
                    None, None, "Hardware configuration keys must be strings", key_node.start_mark
                )
            if key in mapping:
                raise ConstructorError(
                    None, None, f"Duplicate hardware configuration key {key!r}",
                    key_node.start_mark
                )
            mapping[key] = self.construct_object(value_node, deep=deep)
        return mapping


def _fields(value, label, required, optional=()):
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a mapping")
    keys = set(value)
    missing = required - keys
    if missing:
        raise ValueError(f"{label} is missing: {', '.join(sorted(missing))}")
    unknown = keys.difference(required, optional)
    if unknown:
        raise ValueError(f"{label} has unknown keys: {', '.join(sorted(unknown))}")


def _number(value, label, *, positive=False):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{label} must be a finite number")
    try:
        number = float(value)
    except OverflowError as error:
        raise ValueError(f"{label} must be a finite number") from error
    if not math.isfinite(number):
        raise ValueError(f"{label} must be a finite number")
    if positive and number <= 0.0:
        raise ValueError(f"{label} must be positive")
    return number


def _integer(value, label, minimum, maximum):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError(f"{label} must be an integer in [{minimum}, {maximum}]")
    return value


def _angles(values, label):
    try:
        first, second, third = values
    except (TypeError, ValueError) as error:
        raise ValueError(f"{label} must contain three angles in radians") from error
    return tuple(_number(value, label) for value in (first, second, third))


def _joint_order(joint_names):
    try:
        names = tuple(joint_names)
    except TypeError as error:
        raise ValueError("Hardware settings need three nonempty geometry joint names") from error
    if len(names) != 3 or any(not isinstance(name, str) or not name for name in names):
        raise ValueError("Hardware settings need three nonempty geometry joint names")
    if len(set(names)) != 3:
        raise ValueError("The geometry joint names must be distinct")
    return names


def _serial_path(raw_port, joint_name):
    if (not isinstance(raw_port, str) or not raw_port.startswith("/")
            or raw_port != raw_port.strip() or any(ord(char) < 32 for char in raw_port)):
        raise ValueError(f"{joint_name}.port must be an absolute serial device path")
    # Compare paths lexically without opening the ports or resolving device links.
    port = "/" + posixpath.normpath(raw_port).lstrip("/")
    if port == "/":
        raise ValueError(f"{joint_name}.port must name a serial device")
    return port


def _bluetooth_address(value):
    if (not isinstance(value, str) or _BLUETOOTH_MAC.fullmatch(value) is None
            or value.upper() in ("00:00:00:00:00:00", "FF:FF:FF:FF:FF:FF")):
        raise ValueError("bluetooth_address must be a Linux Bluetooth MAC address")
    return value.upper()


def _validate_usb_wiring(axes):
    devices = {(axis.port, axis.address) for axis in axes}
    ports = {axis.port for axis in axes}
    if len(devices) != 3:
        raise ValueError("A serial device path and address may identify only one drum")
    if len(ports) not in (1, 3):
        raise ValueError("Configure one dedicated shared RS485 port or three independent ports")
    if len(ports) == 1 and not any(axis.address == 1 for axis in axes):
        raise ValueError("The shared RS485 port needs drive address 1 for the broadcast reply")


@dataclass(frozen=True)
class AxisSettings:
    """Calibration for one installed motor and drum, in radians and shaft turns."""

    joint_name: str
    port: str | None
    address: int
    pulses_per_revolution: int
    direction: int
    gear_ratio: float
    lower_rad: float
    upper_rad: float
    expected_firmware: int | None
    channel: int | None = None

    @classmethod
    def _read(cls, joint_name, values, allow_motion, transport):
        connection_field = "channel" if transport == "bluetooth" else "port"
        _fields(values, joint_name, _AXIS_FIELDS | {connection_field}, {"expected_firmware"})
        port, channel = None, None
        if transport == "bluetooth":
            channel = _integer(values["channel"], f"{joint_name}.channel", 0, 2)
        else:
            port = _serial_path(values["port"], joint_name)
        address = _integer(values["address"], f"{joint_name}.address", 1, 255)
        pulses = _integer(
            values["pulses_per_revolution"], f"{joint_name}.pulses_per_revolution",
            1, _MAX_PULSES_PER_REVOLUTION
        )
        direction = values["direction"]
        if type(direction) is not int or direction not in (-1, 1):
            raise ValueError(f"{joint_name}.direction must be +1 or -1")
        ratio = _number(values["gear_ratio"], f"{joint_name}.gear_ratio", positive=True)
        lower = _number(values["lower_rad"], f"{joint_name}.lower_rad")
        upper = _number(values["upper_rad"], f"{joint_name}.upper_rad")
        if lower >= upper or lower > 0.0 or upper < 0.0:
            raise ValueError(
                f"{joint_name} travel must be increasing and include the zero position"
            )
        firmware = values.get("expected_firmware")
        if firmware is not None:
            firmware = _integer(firmware, f"{joint_name}.expected_firmware", 0, 255)
        elif allow_motion:
            raise ValueError(
                f"{joint_name}.expected_firmware is required when allow_motion is true"
            )
        return cls(
            joint_name, port, address, pulses, direction, ratio, lower, upper, firmware, channel
        )


@dataclass(frozen=True)
class HardwareSettings:
    """Three hardware axes in geometry joint order and common serial settings."""

    axes: tuple[AxisSettings, AxisSettings, AxisSettings]
    allow_motion: bool
    baudrate: int
    timeout_s: float
    poll_rate_hz: float
    command_rate_hz: float
    command_timeout_s: float
    max_drum_speed_rad_s: float
    max_command_lead_rad: float
    hold_tolerance_rad: float
    acceleration: int
    speeds_rpm: tuple[int, int, int]
    transport: str = "usb"
    bluetooth_address: str | None = None
    bluetooth_connect_timeout_s: float | None = None
    bluetooth_response_timeout_s: float | None = None

    @classmethod
    def from_file(cls, path, joint_names):
        """Load driver settings and validate the full axis assignment."""
        path = Path(path).expanduser()
        try:
            values = yaml.load(path.read_text(encoding="utf-8"), Loader=_UniqueKeysLoader)
            return cls._read(values, _joint_order(joint_names))
        except (OSError, ValueError, yaml.YAMLError) as error:
            raise ValueError(f"Invalid hardware configuration {path}: {error}") from error

    @classmethod
    def _read(cls, values, joint_names):
        _fields(values, "Hardware configuration", {"protocol", "axes"},
                {*_DEFAULTS, "transport", *_BLUETOOTH_FIELDS})
        if values["protocol"] != PROTOCOL:
            raise ValueError(f"protocol must be {PROTOCOL}")
        transport = values.get("transport", "usb")
        if transport not in ("usb", "bluetooth"):
            raise ValueError("transport must be usb or bluetooth")
        bluetooth = {}
        if transport == "bluetooth":
            bluetooth["bluetooth_address"] = _bluetooth_address(values.get("bluetooth_address"))
            for field, default in _BLUETOOTH_TIMEOUTS.items():
                bluetooth[field] = _number(values.get(field, default), field, positive=True)
            options = {**_DEFAULTS, **_BLUETOOTH_DEFAULTS, **values}
        else:
            if _BLUETOOTH_FIELDS.intersection(values):
                raise ValueError("Bluetooth settings require transport: bluetooth")
            options = {**_DEFAULTS, **values}
        allow_motion = options["allow_motion"]
        if type(allow_motion) is not bool:
            raise ValueError("allow_motion must be a boolean")
        rows = values["axes"]
        _fields(rows, "axes", set(joint_names))
        axes = tuple(
            AxisSettings._read(name, rows[name], allow_motion, transport) for name in joint_names
        )
        if transport == "bluetooth":
            if len({axis.channel for axis in axes}) != 3:
                raise ValueError("Bluetooth channels 0, 1 and 2 must each identify one drum")
        else:
            _validate_usb_wiring(axes)

        baudrate = options["baudrate"]
        if type(baudrate) is not int or baudrate not in SUPPORTED_BAUDRATES:
            raise ValueError("baudrate must be a rate supported by Emm V5 UART")
        if transport == "bluetooth" and baudrate != BLUETOOTH_UART_BAUDRATE:
            raise ValueError(
                f"Bluetooth bridge motor UART baudrate must be {BLUETOOTH_UART_BAUDRATE}"
            )
        positive_fields = (
            "timeout_s", "poll_rate_hz", "command_rate_hz", "command_timeout_s",
            "max_drum_speed_rad_s", "max_command_lead_rad", "hold_tolerance_rad"
        )
        limits = {name: _number(options[name], name, positive=True) for name in positive_fields}
        if limits["hold_tolerance_rad"] > limits["max_command_lead_rad"]:
            raise ValueError("hold_tolerance_rad must not exceed max_command_lead_rad")
        if limits["command_timeout_s"] <= 1.0 / limits["command_rate_hz"]:
            raise ValueError("command_timeout_s must exceed the command period")
        acceleration = _integer(options["acceleration"], "acceleration", 0, 255)
        speeds_rpm = tuple(
            cls._quantized_speed(axis, limits["max_drum_speed_rad_s"]) for axis in axes
        )
        limits["max_drum_speed_rad_s"] = min(
            speed * _RAD_S_PER_RPM / axis.gear_ratio for axis, speed in zip(axes, speeds_rpm)
        )
        return cls(
            axes=axes, allow_motion=allow_motion, baudrate=baudrate, acceleration=acceleration,
            speeds_rpm=speeds_rpm, transport=transport, **limits, **bluetooth
        )

    @staticmethod
    def _quantized_speed(axis, drum_speed):
        # Disable the driver's 0.1 RPM setting for this legacy integer RPM profile.
        motor_rpm = drum_speed * axis.gear_ratio / _RAD_S_PER_RPM
        if not math.isfinite(motor_rpm) or motor_rpm > LEGACY_MAX_MOTOR_RPM:
            raise ValueError(f"{axis.joint_name} speed exceeds {LEGACY_MAX_MOTOR_RPM} motor RPM")
        speed = math.floor(motor_rpm)
        if speed * _RAD_S_PER_RPM / axis.gear_ratio > drum_speed:
            speed -= 1
        if speed < 1:
            raise ValueError(f"{axis.joint_name} speed is below the minimum 1 motor RPM")
        return speed

    def check_drums(self, angles) -> Angles:
        """Return finite drum angles inside the calibrated travel limits."""
        angles = _angles(angles, "drum angles")
        for axis, angle in zip(self.axes, angles):
            if not axis.lower_rad <= angle <= axis.upper_rad:
                raise ValueError(f"{axis.joint_name} angle is outside calibrated drum travel")
        return angles

    def to_drum_angles(self, motor_angles, home_motor_angles) -> Angles:
        """Convert measured motor to calibrated drum angles without discarding overtravel."""
        motor = _angles(motor_angles, "motor angles")
        home = _angles(home_motor_angles, "home motor angles")
        drum = tuple(
            axis.direction * (measured - zero) / axis.gear_ratio
            for axis, measured, zero in zip(self.axes, motor, home)
        )
        return _angles(drum, "calculated drum angles")

    def to_motor_angles(self, drum_angles, home_motor_angles) -> Angles:
        """Convert feasible drum commands to the drive's absolute motor coordinates."""
        drum = self.check_drums(drum_angles)
        home = _angles(home_motor_angles, "home motor angles")
        motor = tuple(
            zero + axis.direction * axis.gear_ratio * target
            for axis, target, zero in zip(self.axes, drum, home)
        )
        return _angles(motor, "calculated motor angles")
