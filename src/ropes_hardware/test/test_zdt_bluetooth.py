"""Exercise the calibrated motor protocol across a simulated three-UART BLE gateway."""

import math
from types import SimpleNamespace

import pytest
from ropes_hardware.ble_gateway import BleGatewayError
from ropes_hardware.zdt_bluetooth import BluetoothZdtEmmV5
from ropes_hardware.zdt_emm_v5 import DriveError, MoveInterrupted


class FakeGateway:
    def __init__(self, _address, _connect_timeout, _response_timeout):
        self.events = []
        self.start_mask = 7
        self.stop_mask = 7

    def exchange(self, channel, frame):
        self.events.append(('transfer', channel, frame))
        address, opcode = frame[:2]
        if opcode == 0x42:
            body = bytearray.fromhex(
                '21 15 19 02 02 02 00 10 01 00 03 E8 0B B8 0F A0 '
                '05 07 01 00 01 01 00 28 09 60 0F A0 00 01'
            )
            body[18] = address
        else:
            body = {
                0x1F: b'\x7d\x6f', 0x36: b'\x01\x00\x01\x00\x00',
                0x3A: b'\x01', 0xFD: b'\x02',
            }[opcode]
        return frame[:2] + body + b'\x6b'

    def arm(self, addresses):
        self.events.append(('arm', addresses))

    def start_group(self):
        self.events.append(('start',))
        return self.start_mask

    def keepalive(self):
        self.events.append(('keepalive',))

    def stop_all(self):
        self.events.append(('stop',))
        return self.stop_mask

    def close(self):
        self.events.append(('close',))


@pytest.fixture
def drive_and_gateway():
    axes = tuple(SimpleNamespace(channel=channel, address=channel + 1, pulses_per_revolution=3200)
                 for channel in (2, 0, 1))
    settings = SimpleNamespace(
        axes=axes, baudrate=115200, timeout_s=0.05, bluetooth_address='AA:BB:CC:DD:EE:FF',
        bluetooth_connect_timeout_s=15.0, bluetooth_response_timeout_s=0.4,
    )
    driver = BluetoothZdtEmmV5(settings, gateway_factory=FakeGateway)
    try:
        yield driver, driver._gateway
    finally:
        driver.close()


def move(driver, can_start=None):
    driver.move_absolute((0.0, math.tau, -math.pi), (1, 1, 1), 200, can_start=can_start)


def test_read_only_feedback_and_group_start_use_channel_mapping(drive_and_gateway):
    driver, gateway = drive_and_gateway
    with pytest.raises(DriveError, match='Verify'):
        move(driver)
    driver.verify_configuration()
    driver.keepalive()
    samples = driver.read_state()
    assert [sample.position_rad for sample in samples] == [-math.tau] * 3
    assert {event[0] for event in gateway.events} == {'transfer'}
    assert [event[1] for event in gateway.events[-6:]] == [2, 2, 0, 0, 1, 1]

    gateway.events.clear()
    move(driver)
    assert gateway.events[0] == ('arm', (1, 2, 3))
    assert [event[1] for event in gateway.events[1:-1]] == [2, 0, 1]
    assert all(event[2][1] == 0xFD and event[2][-3:-1] == b'\x01\x01'
               for event in gateway.events[1:-1])
    assert gateway.events[-1] == ('start',)
    driver._last_heartbeat = -10.0
    driver.keepalive()
    assert gateway.events[-1] == ('keepalive',)
    driver.stop_all()
    driver.keepalive()
    assert gateway.events[-1] == ('stop',)


def test_withdrawal_after_stage_never_starts(drive_and_gateway):
    driver, gateway = drive_and_gateway
    driver.verify_configuration()
    decisions = iter([True, False])
    with pytest.raises(MoveInterrupted, match='group start'):
        move(driver, can_start=lambda: next(decisions))
    assert ('start',) not in gateway.events
    assert gateway.events[-1] == ('stop',)
    gateway.events.clear()
    move(driver)
    assert gateway.events[0] == ('arm', (1, 2, 3))
    assert [event[1] for event in gateway.events[1:-1]] == [2, 0, 1]
    assert all(event[2][1] == 0xFD for event in gateway.events[1:-1])
    assert gateway.events[-1] == ('start',)


def test_withdrawal_without_all_stop_acks_faults(drive_and_gateway):
    driver, gateway = drive_and_gateway
    driver.verify_configuration()
    decisions = iter([True, False])
    gateway.stop_mask = 5
    with pytest.raises(DriveError, match=r'withdrawn.*stop.*channels \[1\]') as fault:
        move(driver, can_start=lambda: next(decisions))
    assert type(fault.value) is DriveError
    assert ('start',) not in gateway.events
    with pytest.raises(DriveError, match='Verify'):
        move(driver)


def test_partial_ack_or_link_error_disables_further_motion(drive_and_gateway):
    driver, gateway = drive_and_gateway
    driver.verify_configuration()
    gateway.start_mask = 5
    gateway.stop_mask = 5
    with pytest.raises(DriveError, match=r'group start.*channels \[1\].*stop.*channels \[1\]'):
        move(driver)
    assert gateway.events[-2:] == [('start',), ('stop',)]
    with pytest.raises(DriveError, match='Verify'):
        move(driver)

    def disconnected():
        raise BleGatewayError('BLE disconnected')

    gateway.stop_all = disconnected
    with pytest.raises(DriveError, match='ESP32 stop failed.*disconnected'):
        driver.stop_all()
