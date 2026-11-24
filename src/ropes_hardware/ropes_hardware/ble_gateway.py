"""Synchronous host transport for the three-channel UART gateway over BLE GATT."""

import asyncio
from concurrent.futures import TimeoutError as FutureTimeout
from concurrent.futures import wait as wait_futures
from dataclasses import dataclass, field
import math
import re
from threading import Lock, Thread


SERVICE_UUID = '988ebf10-90b1-4b92-9f35-1322d0bd4600'
RX_UUID = '988ebf10-90b1-4b92-9f35-1322d0bd4601'
TX_UUID = '988ebf10-90b1-4b92-9f35-1322d0bd4602'
_VERSION = 1
_CHANNEL_COUNT = 3
_CONTROL_CHANNEL = 0xFF
_EXCHANGE = 1
_ARM = 2
_STOP = 3
_HELLO = 4
_KEEPALIVE = 5
_START_GROUP = 6
_MAX_MOTOR_FRAME = 13
_MAX_PACKET = 20
_REPLY_HEADER_SIZE = 7
_SCHEDULING_ALLOWANCE_S = 0.25
_MAC_PATTERN = re.compile(r'(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}')
_STATUS_ERRORS = {
    1: 'malformed request',
    2: 'gateway is not armed',
    3: 'UART exchange failed',
    4: 'gateway is busy',
    5: 'unsupported operation or board configuration',
}


class BleGatewayError(OSError):
    """The configured gateway did not complete a valid GATT exchange."""


def _bleak_client(address, **kwargs):
    # The USB backend and simulations do not require Bleak.
    try:
        from bleak import BleakClient
    except ImportError as error:
        raise BleGatewayError(
            "BLE requires Bleak 1.1.1. Install with "
            "/usr/bin/python3 -m pip install --user 'bleak==1.1.1'"
        ) from error
    return BleakClient(address, **kwargs)


def _positive_timeout(value, name):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value <= 0.0):
        raise BleGatewayError(f'{name} must be finite and positive')
    return float(value)


@dataclass
class _PendingReply:
    transaction_id: int
    channel: int
    future: asyncio.Future
    data: bytearray = field(default_factory=bytearray)
    write: asyncio.Task | None = None
    write_error: BleGatewayError | None = None


class BleGateway:
    """Own one connection to a pinned gateway MAC with one exchange in flight."""

    def __init__(self, address, connect_timeout_s, response_timeout_s, *, _client_factory=None):
        if not isinstance(address, str) or not _MAC_PATTERN.fullmatch(address):
            raise BleGatewayError('BLE gateway must be configured with a concrete Bluetooth MAC')
        self._address = address.upper()
        self._connect_timeout = _positive_timeout(connect_timeout_s, 'connect_timeout_s')
        self._response_timeout = _positive_timeout(response_timeout_s, 'response_timeout_s')
        self._cleanup_timeout = min(self._connect_timeout, 2.0)
        self._client_factory = _client_factory or _bleak_client
        self._client = None
        self._write_characteristic = None
        self._pending = None
        self._tx_id = 0
        self._disconnected = False
        self._lifecycle_lock = Lock()
        self._calls = set()
        self._closed = False
        self._loop = asyncio.new_event_loop()
        self._thread = Thread(target=self._run_loop, name='ropes-ble-gateway', daemon=True)
        try:
            self._thread.start()
        except RuntimeError as error:
            self._loop.close()
            raise BleGatewayError(f'Could not start BLE gateway event loop: {error}') from error
        try:
            self._call(self._connect, self._connect_timeout, 'connection setup')
            hello = self._request(_HELLO, _CONTROL_CHANNEL)
            if hello != bytes((_VERSION, _CHANNEL_COUNT)):
                raise BleGatewayError(f'Unsupported BLE gateway HELLO reply {hello.hex(" ")}')
        except (BleGatewayError, KeyboardInterrupt):
            try:
                self.close()
            except BleGatewayError:
                pass  # Preserve the original connection or protocol error.
            raise

    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_forever()
        finally:
            tasks = asyncio.all_tasks(self._loop)
            for task in tasks:
                task.cancel()
            if tasks:
                self._loop.run_until_complete(asyncio.wait(tasks, timeout=_SCHEDULING_ALLOWANCE_S))
            self._loop.close()

    async def _with_timeout(self, operation, timeout, description):
        try:
            return await asyncio.wait_for(operation(), timeout)
        except asyncio.TimeoutError as error:
            raise BleGatewayError(f'BLE {description} timed out') from error

    def _schedule(self, operation, timeout, description):
        return asyncio.run_coroutine_threadsafe(
            self._with_timeout(operation, timeout, description), self._loop
        )

    @staticmethod
    def _wait(future, timeout, description):
        try:
            return future.result(timeout=timeout + _SCHEDULING_ALLOWANCE_S)
        except FutureTimeout as error:
            future.cancel()
            raise BleGatewayError(f'BLE {description} timed out') from error
        except BleGatewayError:
            raise
        except Exception as error:
            raise BleGatewayError(f'BLE {description} failed: {error}') from error

    def _call(self, operation, timeout, description):
        with self._lifecycle_lock:
            if self._closed:
                raise BleGatewayError('BLE gateway is closed')
            future = self._schedule(operation, timeout, description)
            self._calls.add(future)
        try:
            return self._wait(future, timeout, description)
        finally:
            with self._lifecycle_lock:
                self._calls.discard(future)

    async def _connect(self):
        self._client = self._client_factory(
            self._address, disconnected_callback=self._on_disconnect,
            timeout=self._connect_timeout, services=[SERVICE_UUID], pair=False,
        )
        await self._client.connect()
        service = self._client.services.get_service(SERVICE_UUID)
        if service is None:
            raise BleGatewayError(f'BLE gateway service {SERVICE_UUID} is missing')
        rx = service.get_characteristic(RX_UUID)
        tx = service.get_characteristic(TX_UUID)
        if rx is None or 'write' not in rx.properties:
            raise BleGatewayError(f'BLE gateway characteristic {RX_UUID} does not support writes')
        if tx is None or 'notify' not in tx.properties:
            raise BleGatewayError(
                f'BLE gateway characteristic {TX_UUID} does not support notifications'
            )
        self._write_characteristic = rx
        await self._client.start_notify(tx, self._on_notify)
        if self._disconnected or not self._client.is_connected:
            raise BleGatewayError('BLE gateway disconnected during connection setup')

    def _on_disconnect(self, _client):
        self._disconnected = True
        self._fail_pending('BLE gateway disconnected')

    def _fail_pending(self, message):
        pending = self._pending
        if pending is None:
            return
        error = BleGatewayError(message)
        if not pending.future.done():
            pending.future.set_exception(error)
        elif (pending.write is not None and not pending.write.done()
                and pending.write_error is None):
            pending.write_error = error
            pending.write.cancel()

    def _on_notify(self, _sender, data):
        pending = self._pending
        # An unidentifiable notification cannot be attributed to this request.
        if pending is None or pending.future.done() or len(data) < 3:
            return
        transaction_id = int.from_bytes(data[1:3], 'little')
        if transaction_id != pending.transaction_id:
            return
        if not _REPLY_HEADER_SIZE <= len(data) <= _MAX_PACKET:
            self._fail_pending('BLE gateway returned a malformed notification size')
            return
        version, status, channel, offset, more = data[0], *data[3:_REPLY_HEADER_SIZE]
        if (version != _VERSION or channel != pending.channel or offset != len(pending.data)
                or more not in (0, 1)):
            self._fail_pending('BLE gateway returned an invalid version, channel or fragment')
            return
        payload = data[_REPLY_HEADER_SIZE:]
        if status != 0:
            detail = _STATUS_ERRORS.get(status)
            if detail is None or offset or more or payload:
                self._fail_pending(f'BLE gateway returned an invalid error status frame {status}')
            else:
                self._fail_pending(f'BLE gateway rejected operation: {detail} (status {status})')
            return
        if more and (not payload or len(pending.data) + len(payload) > 0xFF):
            self._fail_pending('BLE gateway fragment cannot have a valid continuation')
            return
        pending.data.extend(payload)
        if not more:
            pending.future.set_result(bytes(pending.data))

    async def _exchange(self, op, channel, payload):
        if self._closed:
            raise BleGatewayError('BLE gateway is closed')
        if self._disconnected or not self._client.is_connected:
            raise BleGatewayError('BLE gateway disconnected')
        if self._pending is not None:
            raise BleGatewayError('Another BLE gateway operation is in progress')
        self._tx_id = self._tx_id % 0xFFFF + 1
        packet = (
            bytes((_VERSION, self._tx_id & 0xFF, self._tx_id >> 8, op, channel, len(payload)))
            + payload
        )
        reply = self._loop.create_future()
        pending = _PendingReply(self._tx_id, channel, reply)
        self._pending = pending
        write = self._loop.create_task(
            self._client.write_gatt_char(self._write_characteristic, packet, response=True)
        )
        pending.write = write
        try:
            # A link loss or a malformed reply can end the operation while the write is pending.
            _, data = await asyncio.gather(write, reply)
            return data
        except asyncio.CancelledError:
            if pending.write_error is not None:
                raise pending.write_error
            raise
        finally:
            write.cancel()
            reply.cancel()
            self._pending = None

    def _request(self, op, channel, payload=b''):
        return self._call(
            lambda: self._exchange(op, channel, payload),
            self._response_timeout, f'exchange op {op} channel {channel}',
        )

    def exchange(self, channel, frame):
        """Send one raw UART request to one of the three physical motor channels."""
        if type(channel) is not int or not 0 <= channel < _CHANNEL_COUNT:
            raise BleGatewayError(f'Invalid BLE gateway motor channel {channel!r}')
        if not isinstance(frame, (bytes, bytearray, memoryview)):
            raise BleGatewayError('BLE motor frame must be bytes')
        frame = bytes(frame)
        if not 1 <= len(frame) <= _MAX_MOTOR_FRAME:
            raise BleGatewayError(f'BLE motor frame must contain 1 to {_MAX_MOTOR_FRAME} bytes')
        return self._request(_EXCHANGE, channel, frame)

    def arm(self, addresses):
        """Authorize the configured UART motor addresses with the gateway."""
        try:
            addresses = tuple(addresses)
        except TypeError as error:
            raise BleGatewayError('BLE ARM requires three valid motor addresses') from error
        valid = all(type(address) is int and 1 <= address <= 255 for address in addresses)
        if len(addresses) != _CHANNEL_COUNT or not valid:
            raise BleGatewayError('BLE ARM requires three valid motor addresses')
        self._expect_empty(_ARM, bytes(addresses))

    def keepalive(self):
        """Refresh the armed gateway watchdog."""
        self._expect_empty(_KEEPALIVE)

    def _expect_empty(self, op, payload=b''):
        if self._request(op, _CONTROL_CHANNEL, payload):
            raise BleGatewayError(f'BLE gateway returned unexpected data for control op {op}')

    def start_group(self):
        """Start staged UART movement and return the three-channel hardware ACK mask."""
        return self._ack_mask(_START_GROUP)

    def stop_all(self):
        """Request a physical stop on all gateway channels and return the ACK mask."""
        return self._ack_mask(_STOP)

    def _ack_mask(self, op):
        result = self._request(op, _CONTROL_CHANNEL)
        if len(result) != 1 or result[0] >= (1 << _CHANNEL_COUNT):
            raise BleGatewayError(f'BLE gateway returned an invalid ACK mask for control op {op}')
        return result[0]

    async def _disconnect(self):
        self._fail_pending('BLE gateway is closed')
        if self._client is not None:
            # Bleak automatically stops notifications when the link closes.
            await self._client.disconnect()

    def close(self):
        """Close the link and event loop. Safe to call more than once."""
        with self._lifecycle_lock:
            if self._closed:
                return
            self._closed = True
            outstanding = tuple(call for call in self._calls if not call.done())
            future = self._schedule(self._disconnect, self._cleanup_timeout, 'disconnect')
        try:
            self._wait(future, self._cleanup_timeout, 'disconnect')
        finally:
            # Disconnect wakes pending exchanges. Keep the loop alive until
            # their results have crossed back to the synchronous caller futures.
            _, unfinished = wait_futures(outstanding, timeout=_SCHEDULING_ALLOWANCE_S)
            for call in unfinished:
                call.cancel()
            self._loop.call_soon_threadsafe(self._loop.stop)
            self._thread.join(timeout=_SCHEDULING_ALLOWANCE_S * 2.0)
        if self._thread.is_alive():
            raise BleGatewayError('BLE gateway event loop did not shut down')
