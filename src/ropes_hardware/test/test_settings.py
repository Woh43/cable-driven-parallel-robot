"""Validate hardware settings without opening any serial devices."""

import math
from pathlib import Path

import pytest
import yaml

from ropes_hardware.settings import HardwareSettings, PROTOCOL


JOINTS = ("drum_1_joint", "drum_2_joint", "drum_3_joint")


@pytest.fixture
def configuration():
    return {
        "protocol": PROTOCOL,
        "axes": {
            name: {
                "port": f"/dev/pts/hardware-test-{index}",
                "address": 1,
                "pulses_per_revolution": 3200,
                "direction": (-1 if index == 1 else 1),
                "gear_ratio": (2.0, 1.5, 1.0)[index],
                "lower_rad": -0.8,
                "upper_rad": 0.8,
            }
            for index, name in enumerate(JOINTS)
        },
    }


@pytest.fixture
def bluetooth_configuration(configuration):
    configuration.update(transport="bluetooth", bluetooth_address="12:ab:34:cd:56:ef")
    for channel, axis in enumerate(configuration["axes"].values()):
        del axis["port"]
        axis["channel"] = channel
    return configuration


def _load(tmp_path, options, joint_names=JOINTS):
    path = tmp_path / "hardware.yaml"
    path.write_text(yaml.safe_dump(options), encoding="utf-8")
    return HardwareSettings.from_file(path, joint_names)


def test_independent_channels_follow_geometry_order_with_repeated_ids(tmp_path, configuration):
    settings = _load(tmp_path, configuration, JOINTS[::-1])
    assert tuple(axis.joint_name for axis in settings.axes) == JOINTS[::-1]
    assert tuple(axis.port for axis in settings.axes) == (
        "/dev/pts/hardware-test-2", "/dev/pts/hardware-test-1", "/dev/pts/hardware-test-0"
    )
    assert tuple(axis.address for axis in settings.axes) == (1, 1, 1)
    assert settings.speeds_rpm == (2, 4, 5)
    assert settings.max_drum_speed_rad_s == pytest.approx(2 * math.tau / 60)
    assert not settings.allow_motion
    assert all(axis.expected_firmware is None for axis in settings.axes)


def test_motion_requires_firmware_bytes_for_all_drives(tmp_path, configuration):
    configuration["allow_motion"] = True
    with pytest.raises(ValueError, match="expected_firmware is required"):
        _load(tmp_path, configuration)
    for index, axis in enumerate(configuration["axes"].values()):
        axis["expected_firmware"] = 0xF4 + index
    settings = _load(tmp_path, configuration)
    assert settings.allow_motion
    assert tuple(axis.expected_firmware for axis in settings.axes) == (244, 245, 246)
    configuration["axes"][JOINTS[0]]["expected_firmware"] = 256
    with pytest.raises(ValueError, match="expected_firmware"):
        _load(tmp_path, configuration)


def test_shared_bus_requires_unique_addresses_including_one(tmp_path, configuration):
    for index, axis in enumerate(configuration["axes"].values(), start=1):
        axis["port"] = "/dev/serial/by-id/rs485"
        axis["address"] = index
    assert tuple(axis.address for axis in _load(tmp_path, configuration).axes) == (1, 2, 3)
    configuration["axes"][JOINTS[2]]["port"] = "/dev/serial/by-id/./rs485"
    configuration["axes"][JOINTS[2]]["address"] = 2
    with pytest.raises(ValueError, match="only one drum"):
        _load(tmp_path, configuration)
    configuration["axes"][JOINTS[2]]["address"] = 3
    configuration["axes"][JOINTS[0]]["address"] = 4
    with pytest.raises(ValueError, match="address 1"):
        _load(tmp_path, configuration)


def test_two_physical_ports_are_ambiguous(tmp_path, configuration):
    configuration["axes"][JOINTS[1]].update(port="/dev/pts/hardware-test-0", address=2)
    with pytest.raises(ValueError, match="three independent ports"):
        _load(tmp_path, configuration)


@pytest.mark.parametrize(
    "key,value,reason",
    [
        ("port", "ttyUSB0", "absolute"),
        ("port", "/dev/ttyUSB0\n", "absolute"),
        ("address", 0, "address"),
        ("address", True, "address"),
        ("direction", -1.0, "direction"),
        ("gear_ratio", 0.0, "positive"),
        ("gear_ratio", math.nan, "finite"),
        ("pulses_per_revolution", 0, "pulses_per_revolution"),
        ("lower_rad", 0.1, "include the zero"),
        ("upper_rad", math.inf, "finite"),
        ("expected_firmware", "0xF4", "expected_firmware"),
    ],
)
def test_rejects_invalid_axis_calibration(tmp_path, configuration, key, value, reason):
    configuration["axes"][JOINTS[0]][key] = value
    with pytest.raises(ValueError, match=reason):
        _load(tmp_path, configuration)


@pytest.mark.parametrize(
    "key,value,reason",
    [
        ("protocol", "modbus", PROTOCOL),
        ("allow_motion", 1, "boolean"),
        ("acceleration", -1, "acceleration"),
        ("acceleration", 256, "acceleration"),
        ("baudrate", 230400, "baudrate"),
        ("timeout_s", 0.0, "positive"),
        ("poll_rate_hz", math.nan, "finite"),
        ("command_rate_hz", 0.0, "positive"),
        ("command_timeout_s", 0.01, "command period"),
        ("hold_tolerance_rad", 0.07, "max_command_lead_rad"),
        ("max_drum_speed_rad_s", 0.01, "minimum 1 motor RPM"),
        ("max_drum_speed_rad_s", 1000.0, "exceeds 5000 motor RPM"),
        ("unknown_option", 3, "unknown keys"),
    ],
)
def test_rejects_invalid_common_settings(tmp_path, configuration, key, value, reason):
    configuration[key] = value
    with pytest.raises(ValueError, match=reason):
        _load(tmp_path, configuration)


def test_rejects_missing_and_duplicate_keys(tmp_path, configuration):
    del configuration["axes"][JOINTS[0]]["direction"]
    with pytest.raises(ValueError, match="missing: direction"):
        _load(tmp_path, configuration)
    del configuration["axes"][JOINTS[0]]
    with pytest.raises(ValueError, match=JOINTS[0]):
        _load(tmp_path, configuration)
    path = tmp_path / "duplicates.yaml"
    path.write_text("allow_motion: true\nallow_motion: false\n", encoding="utf-8")
    with pytest.raises(ValueError, match="Duplicate.*allow_motion"):
        HardwareSettings.from_file(path, JOINTS)


def test_converts_calibrated_motor_coordinates_and_reports_overtravel(tmp_path, configuration):
    settings = _load(tmp_path, configuration)
    home = (1.1, -1.8, 0.3)
    desired = (0.25, -0.5, 0.7)
    motor = settings.to_motor_angles(desired, home)
    assert motor == pytest.approx((1.6, -1.05, 1.0))
    assert settings.to_drum_angles(motor, home) == pytest.approx(desired)
    assert settings.to_drum_angles(home, home) == (0.0, 0.0, 0.0)
    assert settings.to_drum_angles((3.1, -1.8, 0.3), home) == pytest.approx((1.0, 0.0, 0.0))
    with pytest.raises(ValueError, match="calibrated drum travel"):
        settings.to_motor_angles((0.81, 0.0, 0.0), home)
    with pytest.raises(ValueError, match="finite"):
        settings.check_drums((math.nan, 0.0, 0.0))
    with pytest.raises(ValueError, match="three angles"):
        settings.to_drum_angles((0.0, 0.0), home)


def test_explicit_usb_keeps_legacy_defaults(tmp_path, configuration):
    legacy = _load(tmp_path, configuration)
    configuration["transport"] = "usb"
    explicit = _load(tmp_path, configuration)
    assert legacy == explicit
    assert legacy.transport == "usb"
    assert legacy.bluetooth_address is None
    assert legacy.bluetooth_connect_timeout_s is None
    assert legacy.bluetooth_response_timeout_s is None
    assert all(axis.channel is None for axis in legacy.axes)


def test_bluetooth_assigns_channels_by_geometry_order(tmp_path, bluetooth_configuration):
    settings = _load(tmp_path, bluetooth_configuration, (JOINTS[2], JOINTS[0], JOINTS[1]))
    assert settings.transport == "bluetooth"
    assert tuple(axis.channel for axis in settings.axes) == (2, 0, 1)
    assert tuple(axis.address for axis in settings.axes) == (1, 1, 1)
    assert all(axis.port is None for axis in settings.axes)
    assert settings.bluetooth_address == "12:AB:34:CD:56:EF"
    assert settings.bluetooth_connect_timeout_s == 15.0
    assert settings.bluetooth_response_timeout_s == 0.40
    assert settings.poll_rate_hz == settings.command_rate_hz == 5.0
    assert settings.command_timeout_s == 0.75
    assert settings.baudrate == 115200
    assert settings.speeds_rpm == (1, 3, 2)


def test_bluetooth_example_is_valid_read_only(tmp_path):
    example = Path(__file__).resolve().parents[1] / "config" / "hardware.bluetooth.example.yaml"
    settings = HardwareSettings.from_file(example, JOINTS)
    assert settings.transport == "bluetooth"
    assert settings.speeds_rpm == (1, 1, 1)
    assert not settings.allow_motion
    assert all(axis.expected_firmware is None for axis in settings.axes)


@pytest.mark.parametrize("value", [None, "invalid", "00:00:00:00:00:00", "ff:ff:ff:ff:ff:ff"])
def test_rejects_invalid_bluetooth_mac(tmp_path, bluetooth_configuration, value):
    bluetooth_configuration["bluetooth_address"] = value
    with pytest.raises(ValueError, match="Bluetooth MAC address"):
        _load(tmp_path, bluetooth_configuration)


@pytest.mark.parametrize("field", ["bluetooth_connect_timeout_s", "bluetooth_response_timeout_s"])
def test_rejects_invalid_bluetooth_timeouts(tmp_path, bluetooth_configuration, field):
    for value in (False, -0.1, math.inf):
        bluetooth_configuration[field] = value
        with pytest.raises(ValueError, match=field):
            _load(tmp_path, bluetooth_configuration)
    bluetooth_configuration[field] = 0.2
    assert getattr(_load(tmp_path, bluetooth_configuration), field) == 0.2


def test_rejects_transport_specific_options_for_other_transport(tmp_path, configuration):
    configuration["transport"] = "bluetooth-classic"
    with pytest.raises(ValueError, match="transport must"):
        _load(tmp_path, configuration)
    configuration.update(transport="usb", bluetooth_address="02:00:00:00:00:00")
    with pytest.raises(ValueError, match="Bluetooth settings require"):
        _load(tmp_path, configuration)


def test_bluetooth_requires_bridge_baudrate_and_distinct_channels(
    tmp_path, bluetooth_configuration,
):
    bluetooth_configuration["baudrate"] = 57600
    with pytest.raises(ValueError, match="baudrate must be 115200"):
        _load(tmp_path, bluetooth_configuration)
    bluetooth_configuration["baudrate"] = 115200
    axis = bluetooth_configuration["axes"][JOINTS[0]]
    for value in (-1, 3, True):
        axis["channel"] = value
        with pytest.raises(ValueError, match="channel must be an integer"):
            _load(tmp_path, bluetooth_configuration)
    axis["channel"] = 1
    with pytest.raises(ValueError, match="must each identify one"):
        _load(tmp_path, bluetooth_configuration)
    axis.update(channel=0, port="/dev/ttyUSB0")
    with pytest.raises(ValueError, match="unknown keys: port"):
        _load(tmp_path, bluetooth_configuration)
