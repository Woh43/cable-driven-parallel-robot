"""Use the ESP32's three UART channels with the existing Emm V5 drive protocol."""

from time import monotonic

from .ble_gateway import BleGateway
from .zdt_emm_v5 import DriveError, MoveInterrupted, ZdtEmmV5


_ALL_AXES = 0b111
_HEARTBEAT_PERIOD_S = 0.18


class _GatewayUart:
    """Buffer one complete gateway response for the normal Emm V5 frame reader."""

    def __init__(self, gateway, channel):
        self._gateway = gateway
        self._channel = channel
        self._reply = b''
        self.timeout = 0.0

    def reset_input_buffer(self):
        self._reply = b''

    def write(self, data):
        self._reply = self._gateway.exchange(self._channel, data)
        return len(data)

    def read(self, size):
        result, self._reply = self._reply[:size], self._reply[size:]
        return result

    def close(self):
        self.reset_input_buffer()


class BluetoothZdtEmmV5(ZdtEmmV5):
    """Keep host-side calibration and validation while grouping starts on the ESP32."""

    def __init__(self, settings, *, gateway_factory=BleGateway):
        axes = settings.axes
        channels = {f'BLE UART{axis.channel}': axis.channel for axis in axes}
        self._channel_addresses = tuple(
            axis.address for axis in sorted(axes, key=lambda item: item.channel)
        )
        self._gateway_armed = False
        self._last_heartbeat = 0.0
        self._gateway = gateway_factory(
            settings.bluetooth_address, settings.bluetooth_connect_timeout_s,
            settings.bluetooth_response_timeout_s,
        )
        try:
            super().__init__(
                tuple(channels), tuple(axis.address for axis in axes),
                tuple(axis.pulses_per_revolution for axis in axes),
                settings.baudrate, settings.timeout_s,
                port_factory=lambda port, _baud, _timeout: _GatewayUart(
                    self._gateway, channels[port]
                ),
            )
        except BaseException:
            self._close_gateway()
            raise

    def _prepare_motion(self, can_start):
        if can_start is not None and not can_start():
            raise MoveInterrupted('Motion authorization withdrawn before gateway staging')
        if not self._gateway_armed:
            try:
                self._gateway.arm(self._channel_addresses)
            except OSError as error:
                raise DriveError(f'ESP32 arm failed: {error}') from error
            self._gateway_armed = True
            self._last_heartbeat = monotonic()

    def _start_group(self, can_start):
        if can_start is not None and not can_start():
            raise MoveInterrupted('Motion authorization withdrawn before ESP32 group start')
        try:
            mask = self._gateway.start_group()
        except OSError as error:
            raise DriveError(f'ESP32 group start failed: {error}') from error
        self._check_mask(mask, 'group start')

    def keepalive(self):
        """Renew the ESP32 movement lease only while the ROS hardware gate is healthy."""
        if not self._gateway_armed or monotonic() - self._last_heartbeat < _HEARTBEAT_PERIOD_S:
            return
        self._gateway.keepalive()
        self._last_heartbeat = monotonic()

    def stop_all(self):
        if not self._verified:
            raise DriveError('No verified drives to stop')
        try:
            self._check_mask(self._gateway.stop_all(), 'stop')
        except OSError as error:
            self._requires_reverify = True
            raise DriveError(f'ESP32 stop failed: {error}') from error
        except DriveError:
            self._requires_reverify = True
            raise
        finally:
            self._gateway_armed = False

    @staticmethod
    def _check_mask(mask, operation):
        if mask != _ALL_AXES:
            missing = [channel for channel in range(3) if not mask & (1 << channel)]
            raise DriveError(f'ESP32 {operation} was not acknowledged on channels {missing}')

    def _close_gateway(self):
        gateway, self._gateway = self._gateway, None
        if gateway is not None:
            gateway.close()

    def close(self):
        super().close()
        self._close_gateway()
