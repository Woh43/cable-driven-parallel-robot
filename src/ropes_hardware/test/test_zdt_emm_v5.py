"""Check Rev1.3 UART framing and queuing with simulated serial endpoints."""

import math
import termios
from types import SimpleNamespace

import pytest

from ropes_hardware import zdt_emm_v5 as emm


class FakeSerial:
    def __init__(self, name, timeout, trace, respond):
        self.name = name
        self.timeout = timeout
        self.trace = trace
        self.respond = respond
        self.data = b''
        self.last_command = None
        self.closed = False

    def reset_input_buffer(self):
        self.data = b''

    def write(self, frame):
        self.last_command = frame[1]
        self.trace.append(('write', self.name, frame))
        self.data += self.respond(self.name, frame)
        return len(frame)

    def read(self, _size):
        self.trace.append(('read', self.name, self.last_command))
        byte, self.data = self.data[:1], self.data[1:]
        return byte

    def close(self):
        self.closed = True


def _reply(address, command, data):
    return bytes((address, command)) + data + b'\x6b'


def _configuration(address, microsteps=16, mode=1):
    fields = bytearray.fromhex(
        '21 15 19 02 02 02 00 10 01 00 03 E8 0B B8 0F A0 '
        '05 07 01 00 01 01 00 28 09 60 0F A0 00 01')
    fields[7] = microsteps
    fields[18] = address
    fields[20] = mode
    return bytes(fields)


def _respond(_port, frame):
    address, command = frame[:2]
    if command == 0x1F:
        return _reply(address, command, b'\x7d\x6f')
    if command == 0x42:
        return _reply(address, command, _configuration(address))
    if command == 0x36:
        return _reply(address, command, bytes.fromhex('01 00 01 00 00'))
    if command == 0x3A:
        return _reply(address, command, b'\x01')
    return _reply(address or 1, command, b'\x02')


DISTINCT = ('/fake/first', '/fake/second', '/fake/third')
SHARED = ('/fake/shared',) * 3


def _open_fake(monkeypatch, respond=_respond):
    handles = {}
    trace = []

    def open_port(port, baudrate, timeout):
        assert port not in handles
        assert baudrate == 115200 and timeout == 0.05
        handles[port] = FakeSerial(port, timeout, trace, respond)
        return handles[port]

    monkeypatch.setattr(emm, '_serial_port', open_port)
    return handles, trace


def _writes(trace):
    return [(port, value) for kind, port, value in trace if kind == 'write']


def _move(driver, **kwargs):
    driver.move_absolute((math.tau, -math.tau / 2, 0.0), (60, 25, 7), 20, **kwargs)


def test_read_only_legacy_checks_and_signed_multiturn_feedback(monkeypatch):
    def respond(port, frame):
        if frame[1] == 0x36 and port == DISTINCT[2]:
            return _reply(frame[0], 0x36, bytes.fromhex('00 00 03 6B 00'))
        if frame[1] == 0x3A and port == DISTINCT[2]:
            return _reply(frame[0], 0x3A, b'\x08')
        return _respond(port, frame)

    handles, trace = _open_fake(monkeypatch, respond)
    driver = emm.ZdtEmmV5(DISTINCT, (1, 1, 1), (3200,) * 3, 115200, 0.05)
    with pytest.raises(emm.DriveError, match='Verify'):
        _move(driver)
    assert not trace
    assert driver.verify_configuration() == ((0x7D, 0x6F),) * 3
    states = driver.read_state()
    assert states[:2] == (emm.MotorSample(-math.tau, True, False),) * 2
    assert states[2].position_rad == pytest.approx(0x036B00 * math.tau / 65536)
    assert states[2].stalled and not states[2].enabled
    assert [frame[1] for _, frame in _writes(trace)] == [0x1F] * 3 + [0x42] * 3 + [0x36, 0x3A] * 3
    driver.close()
    assert all(port.closed for port in handles.values())


def test_shared_bus_ack_and_exact_staged_absolute_packets(monkeypatch):
    handles, trace = _open_fake(monkeypatch)
    driver = emm.ZdtEmmV5(SHARED, (3, 1, 2), (3200,) * 3, 115200, 0.05)
    driver.verify_configuration()
    trace.clear()
    _move(driver)
    assert len(handles) == 1
    assert [frame for _, frame in _writes(trace)] == [
        bytes.fromhex('03 FD 00 00 3C 14 00 00 0C 80 01 01 6B'),
        bytes.fromhex('01 FD 01 00 19 14 00 00 06 40 01 01 6B'),
        bytes.fromhex('02 FD 00 00 07 14 00 00 00 00 01 01 6B'),
        bytes.fromhex('00 FF 66 6B'),
    ]


def test_distinct_ports_trigger_and_stop_all_before_reading_any_replies(monkeypatch):
    _, trace = _open_fake(monkeypatch)
    driver = emm.ZdtEmmV5(DISTINCT, (1, 1, 1), (3200,) * 3, 115200, 0.05)
    driver.verify_configuration()
    trace.clear()
    _move(driver)
    driver.stop_all()
    for command, request in ((0xFF, '01 FF 66 6B'), (0xFE, '01 FE 98 00 6B')):
        events = [event for event in trace
                  if event[2] == command or event[2] == bytes.fromhex(request)]
        assert events[:3] == [('write', port, bytes.fromhex(request)) for port in DISTINCT]
        assert all(event[0] == 'read' for event in events[3:])


def test_unresponsive_channel_does_not_hide_other_stop_acks(monkeypatch):
    handles, _ = _open_fake(monkeypatch)
    driver = emm.ZdtEmmV5(DISTINCT, (1, 1, 1), (3200,) * 3, 115200, 0.05)
    driver.verify_configuration()
    now = 123.0
    monkeypatch.setattr(emm, 'time', SimpleNamespace(monotonic=lambda: now))

    def timeout(_):
        nonlocal now
        now += handles[DISTINCT[1]].timeout + 0.001
        return b''

    handles[DISTINCT[1]].read = timeout
    with pytest.raises(emm.DriveError) as fault:
        driver.stop_all()
    assert DISTINCT[1] in str(fault.value)
    assert DISTINCT[0] not in str(fault.value)
    assert DISTINCT[2] not in str(fault.value)


def test_invalid_layout_or_nreceive_never_sends_motion(monkeypatch):
    def respond(port, frame):
        if frame[1] == 0x42:
            return _reply(frame[0], 0x42, _configuration(frame[0], mode=3))
        return _respond(port, frame)

    _, trace = _open_fake(monkeypatch, respond)
    driver = emm.ZdtEmmV5(DISTINCT, (1, 1, 1), (3200,) * 3, 115200, 0.05)
    with pytest.raises(emm.DriveError, match='nReceive'):
        driver.verify_configuration()
    with pytest.raises(emm.DriveError, match='Verify'):
        _move(driver)
    with pytest.raises(emm.DriveError, match='No verified'):
        driver.stop_all()
    assert {frame[1] for _, frame in _writes(trace)} == {0x1F, 0x42}


def test_rejected_stage_stops_all_and_never_triggers(monkeypatch):
    def respond(port, frame):
        if frame[:2] == bytes((2, 0xFD)):
            return _reply(2, 0xFD, b'\xe2')
        if frame[:2] == bytes((1, 0xFE)):
            return b''
        return _respond(port, frame)

    _, trace = _open_fake(monkeypatch, respond)
    driver = emm.ZdtEmmV5(SHARED, (1, 2, 3), (3200,) * 3, 115200, 0.05)
    driver.verify_configuration()
    trace.clear()
    with pytest.raises(emm.DriveError, match='E2.*Stop could not be confirmed'):
        _move(driver)
    assert [tuple(frame[:2]) for _, frame in _writes(trace)] == [
        (1, 0xFD), (2, 0xFD), (1, 0xFE), (2, 0xFE), (3, 0xFE),
    ]
    with pytest.raises(emm.DriveError, match='Verify'):
        _move(driver)


@pytest.mark.parametrize('trigger_count', [0, 1])
def test_authorization_withdrawal_blocks_next_trigger_and_stops_all(monkeypatch, trigger_count):
    _, trace = _open_fake(monkeypatch)
    driver = emm.ZdtEmmV5(DISTINCT, (1, 1, 1), (3200,) * 3, 115200, 0.05)
    driver.verify_configuration()
    trace.clear()
    decisions = iter([True] * trigger_count + [False])
    with pytest.raises(emm.MoveInterrupted, match='withdrawn'):
        _move(driver, can_start=lambda: next(decisions))
    writes = _writes(trace)
    assert [frame[1] for _, frame in writes] == [0xFD] * 3 + [0xFF] * trigger_count + [0xFE] * 3
    assert [port for port, _ in writes[-3:]] == list(DISTINCT)
    trace.clear()
    _move(driver)
    assert [frame[1] for _, frame in _writes(trace)] == [0xFD] * 3 + [0xFF] * 3


def test_authorization_withdrawal_without_all_stop_acks_latches_transport_fault(monkeypatch):
    def respond(port, frame):
        if port == DISTINCT[0] and frame[1] == 0xFE:
            return b''
        return _respond(port, frame)

    _, trace = _open_fake(monkeypatch, respond)
    driver = emm.ZdtEmmV5(DISTINCT, (1, 1, 1), (3200,) * 3, 115200, 0.05)
    driver.verify_configuration()
    trace.clear()
    with pytest.raises(emm.DriveError, match='withdrawn.*Stop could not be confirmed') as fault:
        _move(driver, can_start=lambda: False)
    assert type(fault.value) is emm.DriveError
    assert [frame[1] for _, frame in _writes(trace)] == [0xFD] * 3 + [0xFE] * 3
    with pytest.raises(emm.DriveError, match='Verify'):
        _move(driver)


def test_malformed_read_is_rejected(monkeypatch):
    def respond(port, frame):
        if frame[1] == 0x36:
            return _reply(frame[0], 0x36, b'\x00' * 5)[:-1] + b'\x00'
        return _respond(port, frame)

    _open_fake(monkeypatch, respond)
    driver = emm.ZdtEmmV5(DISTINCT, (1, 1, 1), (3200,) * 3, 115200, 0.05)
    with pytest.raises(emm.DriveError, match='terminator'):
        driver.read_state()


def test_unplugged_first_port_does_not_block_stops_on_remaining_ports(monkeypatch):
    handles, trace = _open_fake(monkeypatch)
    driver = emm.ZdtEmmV5(DISTINCT, (1, 1, 1), (3200,) * 3, 115200, 0.05)
    driver.verify_configuration()
    trace.clear()

    def unplugged():
        raise termios.error(5, 'Input/output error')

    handles[DISTINCT[0]].reset_input_buffer = unplugged
    with pytest.raises(emm.DriveError, match='write failed for 0x36'):
        driver.read_state()
    with pytest.raises(emm.DriveError) as fault:
        driver.stop_all()
    assert 'Stop could not be confirmed' in str(fault.value)
    assert DISTINCT[0] in str(fault.value)
    assert _writes(trace) == [(port, bytes.fromhex('01 FE 98 00 6B')) for port in DISTINCT[1:]]
