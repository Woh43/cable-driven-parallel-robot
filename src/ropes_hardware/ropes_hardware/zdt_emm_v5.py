"""Legacy Emm_V5 UART transport (Emm_V5 Rev1.3, fixed 0x6B terminator)."""

from dataclasses import dataclass
from itertools import zip_longest
import math
import os
import termios
import time
from typing import Callable


_FRAME_END = 0x6B
_VERSION = 0x1F
_POSITION = 0x36
_STATUS = 0x3A
_CONFIGURATION = 0x42
_MOVE = 0xFD
_STOP = 0xFE
_SYNC = 0xFF
_ACK = 0x02
_ACTUAL_COUNTS_PER_REV = 65536


class DriveError(RuntimeError):
    """The configured drive did not complete an expected UART exchange."""


class MoveInterrupted(DriveError):
    """The caller withdrew start permission and all drives acknowledged the stop."""


@dataclass(frozen=True)
class MotorSample:
    """Measured motor shaft angle and driver state."""

    position_rad: float
    enabled: bool
    stalled: bool


def _serial_port(port, baudrate, timeout_s):
    try:
        import serial
    except ImportError as error:
        raise DriveError('The python3-serial package is required for Emm V5 UART') from error
    return serial.Serial(
        port=port, baudrate=baudrate, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE, timeout=timeout_s, write_timeout=timeout_s, exclusive=True,
        xonxoff=False, rtscts=False, dsrdtr=False,
    )


class ZdtEmmV5:
    """Own the axes on a dedicated UART or RS485 link per configured serial path.

    This transport is called by one I/O owner. On a shared bus the staged movement starts
    with one broadcast. On independent links the host queues a trigger for each link.
    """

    def __init__(self, ports, addresses, pulses_per_revolution, baudrate, timeout_s, *,
                 port_factory=None):
        ports = tuple(ports)
        addresses = tuple(addresses)
        pulses_per_revolution = tuple(pulses_per_revolution)
        if any(len(values) != 3 for values in (ports, addresses, pulses_per_revolution)):
            raise ValueError('Exactly three ports, addresses and pulse resolutions are required')
        if any(not isinstance(port, str) or not port.strip() for port in ports):
            raise ValueError('Each axis requires an explicit serial port path')
        if any(type(addr) is not int or not 1 <= addr <= 255 for addr in addresses):
            raise ValueError('Drive addresses must be integers between 1 and 255')
        if any(type(pulses) is not int or not 1 <= pulses <= 102400
               for pulses in pulses_per_revolution):
            raise ValueError('Pulse resolutions must be positive integers at most 102400 per turn')
        if type(baudrate) is not int or baudrate <= 0:
            raise ValueError('baudrate must be a positive integer')
        if (isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float))
                or not math.isfinite(timeout_s) or timeout_s <= 0):
            raise ValueError('timeout_s must be finite and positive')

        if port_factory is None:
            canonical_ports = tuple(os.path.realpath(port) for port in ports)
            port_factory = _serial_port
        else:
            canonical_ports = ports
        path_by_key = dict(zip(canonical_ports, ports))
        addresses_by_port = {port: [] for port in path_by_key}
        for port, address in zip(canonical_ports, addresses):
            if address in addresses_by_port[port]:
                raise ValueError(f'Drive address {address} is duplicated on {path_by_key[port]}')
            addresses_by_port[port].append(address)
        # The vendor documents that only address 1 acknowledges a broadcast.
        if any(len(ids) > 1 and 1 not in ids for ids in addresses_by_port.values()):
            raise ValueError('Each shared RS485 bus must include drive address 1 for sync ACK')

        self._paths = path_by_key
        self._stop_order = tuple(
            tuple((port, address) for address in ids) for port, ids in addresses_by_port.items()
        )
        self._axes = tuple(zip(canonical_ports, addresses, pulses_per_revolution))
        self._triggers = tuple(
            (port, 0, (0, 1)) if len(ids) > 1 else (port, ids[0], (ids[0],))
            for port, ids in addresses_by_port.items()
        )
        self._timeout_s = float(timeout_s)
        self._serial = {}
        self._verified = False
        self._requires_reverify = False
        try:
            for key, path in self._paths.items():
                self._serial[key] = port_factory(path, baudrate, self._timeout_s)
        except (OSError, termios.error, ValueError, DriveError) as error:
            self.close()
            raise DriveError(f'Cannot open {path}: {error}') from error

    def _send(self, port, address, command, payload=b'', *, can_start=None):
        connection = self._serial.get(port)
        if connection is None:
            raise DriveError('Drive serial transport is closed')
        frame = bytes((address, command)) + payload + bytes((_FRAME_END,))
        try:
            connection.reset_input_buffer()
            if can_start is not None and not can_start():
                raise MoveInterrupted('Motion authorization withdrawn before sync')
            if connection.write(frame) != len(frame):
                raise DriveError(f'{self._paths[port]}: incomplete write of 0x{command:02X}')
        except (OSError, termios.error) as error:
            # PySerial's POSIX input flush can raise a bare termios error on unplug.
            raise DriveError(
                f'{self._paths[port]}: write failed for 0x{command:02X}: {error}') from error
        return time.monotonic() + self._timeout_s

    @staticmethod
    def _read_exact(connection, size, deadline):
        data = bytearray()
        while len(data) < size:
            # Independent links may already have replied while another link was waiting.
            connection.timeout = max(0.0, deadline - time.monotonic())
            chunk = connection.read(size - len(data))
            if not chunk:
                break
            data.extend(chunk)
        return bytes(data)

    def _receive(self, port, addresses, command, payload_size, deadline):
        connection = self._serial[port]
        location = f'{self._paths[port]} (address {addresses[-1]}, command 0x{command:02X})'
        try:
            header = self._read_exact(connection, 2, deadline)
            if len(header) != 2:
                raise DriveError(f'{location}: response timeout')
            if header[0] not in addresses or header[1] not in (0, command):
                raise DriveError(f'{location}: unexpected response header {header.hex(" ")}')
            size = 2 if header[1] == 0 else payload_size + 1
            body = self._read_exact(connection, size, deadline)
        except (OSError, termios.error) as error:
            raise DriveError(f'{location}: serial read failed: {error}') from error
        if len(body) != size:
            raise DriveError(f'{location}: incomplete response')
        if body[-1] != _FRAME_END:
            raise DriveError(f'{location}: invalid 0x6B frame terminator')
        if header[1] == 0:
            raise DriveError(f'{location}: protocol error 0x{body[0]:02X}')
        return body[:-1]

    def _exchange(self, port, address, command, payload_size, request=b''):
        deadline = self._send(port, address, command, request)
        return self._receive(port, (address,), command, payload_size, deadline)

    @staticmethod
    def _require_ack(payload, address, command):
        if payload[0] != _ACK:
            raise DriveError(
                f'Drive {address} rejected 0x{command:02X} with status 0x{payload[0]:02X}')

    def read_firmware(self):
        """Read (firmware byte, hardware byte) pairs in configured axis order."""
        return tuple(tuple(self._exchange(port, address, _VERSION, 2))
                     for port, address, _ in self._axes)

    def verify_configuration(self):
        """Read and validate the documented legacy layout before accepting motion."""
        self._requires_reverify = True
        versions = self.read_firmware()
        for port, address, pulses in self._axes:
            values = self._exchange(port, address, _CONFIGURATION, 30, b'\x6c')
            if values[:2] != b'\x21\x15':
                raise DriveError(f'Drive {address} does not report the legacy Emm V5 layout')
            steps_per_turn = {25: 200, 50: 400}.get(values[2])
            actual_pulses = (values[7] or 256) * steps_per_turn if steps_per_turn else None
            if actual_pulses != pulses:
                raise DriveError(
                    f'Drive {address} reports {actual_pulses} pulses/revolution, '
                    f'configured {pulses}')
            if (values[4], values[18], values[19], values[20]) != (2, address, 0, 1):
                raise DriveError(
                    f'Drive {address} requires UART_FUN, configured address, 0x6B and nReceive')
        self._verified = True
        self._requires_reverify = False
        return versions

    def read_state(self):
        """Read multirevolution encoder angles and enable/stall flags for the three axes."""
        samples = []
        for port, address, _ in self._axes:
            position = self._exchange(port, address, _POSITION, 5)
            if position[0] not in (0, 1):
                raise DriveError(f'Drive {address} sent an invalid sign for actual position')
            angle = int.from_bytes(position[1:], 'big') * math.tau / _ACTUAL_COUNTS_PER_REV
            if position[0]:
                angle = -angle
            flags = self._exchange(port, address, _STATUS, 1)[0]
            samples.append(MotorSample(angle, bool(flags & 0x01), bool(flags & 0x0C)))
        return tuple(samples)

    @staticmethod
    def _position_payload(angle, speed, acceleration, pulses_per_revolution):
        if (isinstance(angle, bool) or not isinstance(angle, (int, float))
                or not math.isfinite(angle)):
            raise ValueError('Motor coordinates must be finite signed angles in radians')
        if (isinstance(speed, bool) or not isinstance(speed, (int, float))
                or not math.isfinite(speed) or not 1 <= speed <= 5000):
            raise ValueError('Motor speeds must be between 1 and 5000 RPM')
        counts = abs(angle) / math.tau * pulses_per_revolution
        if not counts < 2 ** 32 - 0.5:
            raise ValueError('Absolute motor coordinate exceeds the 32-bit pulse range')
        steps = round(counts)
        return (bytes((int(angle < 0.0 and steps != 0),)) + round(speed).to_bytes(2, 'big')
                + bytes((acceleration,)) + steps.to_bytes(4, 'big') + b'\x01\x01')

    def _prepare_motion(self, can_start):
        """Allow alternate links to authorize the upcoming stage operation."""

    def _start_group(self, can_start):
        triggers = []
        for port, address, addresses in self._triggers:
            deadline = self._send(port, address, _SYNC, b'\x66', can_start=can_start)
            triggers.append((port, addresses, deadline))
        for port, addresses, deadline in triggers:
            payload = self._receive(port, addresses, _SYNC, 1, deadline)
            self._require_ack(payload, addresses[-1], _SYNC)

    def move_absolute(
        self, positions_rad, speeds_rpm, acceleration, *,
        can_start: Callable[[], bool] | None = None,
    ):
        """Stage three goals, then trigger only after ACKs and optional caller confirmation."""
        if not self._verified or self._requires_reverify:
            raise DriveError('Verify the legacy drive configuration before commanding motion')
        targets = tuple(positions_rad)
        speeds = tuple(speeds_rpm)
        if len(targets) != 3 or len(speeds) != 3:
            raise ValueError('All three motor positions and speeds are required')
        if type(acceleration) is not int or not 0 <= acceleration <= 255:
            raise ValueError('Drive acceleration must be an integer between 0 and 255')
        payloads = tuple(self._position_payload(angle, speed, acceleration, pulses)
                         for angle, speed, (_, _, pulses) in zip(targets, speeds, self._axes))
        try:
            self._prepare_motion(can_start)
            for (port, address, _), payload in zip(self._axes, payloads):
                self._require_ack(
                    self._exchange(port, address, _MOVE, 1, payload), address, _MOVE)
            self._start_group(can_start)
        except DriveError as error:
            self._requires_reverify = True
            try:
                self.stop_all()
            except DriveError as stop_error:
                raise DriveError(f'{error}; emergency stop feedback: {stop_error}') from error
            if isinstance(error, MoveInterrupted):
                # A later move must overwrite and acknowledge all three staged goals.
                self._requires_reverify = False
            raise

    def stop_all(self):
        """Attempt an immediate stop on every confirmed drive, leaving enable unchanged."""
        if not self._verified:
            raise DriveError('No verified drives to stop')
        failures = []
        # Send across independent links before waiting on any one reply. On a shared bus,
        # complete an addressed exchange before sending another to avoid reply collisions.
        for axes in zip_longest(*self._stop_order):
            pending = []
            for port, address in filter(None, axes):
                try:
                    pending.append((port, address, self._send(port, address, _STOP, b'\x98\x00')))
                except DriveError as error:
                    failures.append(str(error))
            for port, address, deadline in pending:
                try:
                    payload = self._receive(port, (address,), _STOP, 1, deadline)
                    self._require_ack(payload, address, _STOP)
                except DriveError as error:
                    failures.append(str(error))
        if failures:
            self._requires_reverify = True
            raise DriveError('Stop could not be confirmed for every drive: ' + '; '.join(failures))

    def close(self):
        """Close every opened port without writing to any drive."""
        for connection in self._serial.values():
            try:
                connection.close()
            except (OSError, termios.error):
                pass
        self._serial.clear()
        self._verified = False
