import asyncio
from threading import Event, Thread

import pytest

from ropes_hardware.ble_gateway import (
    BleGateway, BleGatewayError, RX_UUID, SERVICE_UUID, TX_UUID,
)


_MAC = 'AE:12:34:56:78:90'


class _Characteristic:
    def __init__(self, uuid, properties):
        self.uuid = uuid
        self.properties = properties


class _Client:
    def __init__(self):
        self.rx = _Characteristic(RX_UUID, ['write'])
        self.tx = _Characteristic(TX_UUID, ['notify'])
        self.services = self
        self.has_service = True
        self.connect_hangs = False
        self.notify_hangs = False
        self.is_connected = False
        self.connect_calls = 0
        self.close_calls = 0
        self.writes = []
        self.hello = b'\x01\x03'
        self.raw_reply = None
        self.control_replies = {2: b'', 3: b'\x05', 5: b'', 6: b'\x07'}

    def factory(self, address, **kwargs):
        assert address == _MAC
        assert kwargs['services'] == [SERVICE_UUID]
        assert kwargs['pair'] is False
        self.disconnect_callback = kwargs['disconnected_callback']
        return self

    async def connect(self):
        self.connect_calls += 1
        self.loop = asyncio.get_running_loop()
        if self.connect_hangs:
            await asyncio.Event().wait()
        self.is_connected = True

    def get_service(self, uuid):
        assert uuid == SERVICE_UUID
        return self if self.has_service else None

    def get_characteristic(self, uuid):
        return {RX_UUID: self.rx, TX_UUID: self.tx}[uuid]

    async def start_notify(self, characteristic, callback):
        assert characteristic is self.tx
        if self.notify_hangs:
            await asyncio.Event().wait()
        self.notify = callback

    def respond(self, transaction, channel, body=b'', *, version=1, status=0, offset=0, more=0):
        prefix = bytes((version, transaction & 0xFF, transaction >> 8,
                        status, channel, offset, more))
        self.notify(self.tx, bytearray(prefix + body))

    async def write_gatt_char(self, characteristic, data, response):
        assert characteristic is self.rx
        assert response is True
        assert data[0] == 1 and data[5] == len(data[6:]) and len(data) <= 20
        self.writes.append(data)
        transaction = int.from_bytes(data[1:3], 'little')
        op, channel = data[3:5]
        if op == 4:
            self.respond(transaction, channel, self.hello)
        elif op in self.control_replies:
            self.respond(transaction, channel, self.control_replies[op])
        elif self.raw_reply is not None:
            await self.raw_reply(transaction, channel)
        else:
            self.respond(transaction, channel, b'\x01\x36\x6b')

    def simulate_disconnect(self):
        self.is_connected = False
        self.disconnect_callback(self)

    async def disconnect(self):
        self.close_calls += 1
        self.simulate_disconnect()


@pytest.fixture
def connection(request):
    client = _Client()
    response_timeout = getattr(request, 'param', 0.12)
    gateway = BleGateway(_MAC.lower(), 0.2, response_timeout, _client_factory=client.factory)
    yield gateway, client
    gateway.close()


def test_wire_control_requests_and_ack_masks(connection):
    gateway, client = connection
    gateway.arm((1, 7, 255))
    gateway.keepalive()
    assert gateway.start_group() == 7
    assert gateway.stop_all() == 5
    assert [value[3:] for value in client.writes] == [
        bytes((4, 255, 0)), bytes((2, 255, 3, 1, 7, 255)), bytes((5, 255, 0)),
        bytes((6, 255, 0)), bytes((3, 255, 0)),
    ]
    gateway.close()
    gateway.close()
    assert client.connect_calls == client.close_calls == 1
    assert not gateway._thread.is_alive()
    with pytest.raises(BleGatewayError, match='closed'):
        gateway.keepalive()


def test_fragmented_reply_and_stale_id(connection):
    gateway, client = connection
    expected = bytes(range(32))

    async def reply(transaction, channel):
        client.respond(transaction - 1, channel, b'stale', version=99, status=99)
        client.respond(transaction, channel, expected[:13], more=1)
        client.respond(transaction, channel, expected[13:26], offset=13, more=1)
        client.respond(transaction, channel, expected[26:], offset=26)

    client.raw_reply = reply
    assert gateway.exchange(2, b'\x01\x36\x6b') == expected
    assert client.writes[-1][3:] == bytes((1, 2, 3, 1, 0x36, 0x6b))


@pytest.mark.parametrize('reply_fields, pattern', [
    ({'offset': 1}, 'invalid version, channel or fragment'),
    ({'more': 2}, 'invalid version, channel or fragment'),
    ({'version': 2}, 'invalid version, channel or fragment'),
    ({'channel': 1}, 'invalid version, channel or fragment'),
    ({'body': b'x' * 14}, 'malformed notification size'),
    ({'more': 1}, 'valid continuation'),
    ({'status': 1}, 'malformed request'),
    ({'status': 2}, 'not armed'),
    ({'status': 3}, 'UART exchange failed'),
    ({'status': 4}, 'gateway is busy'),
    ({'status': 5}, 'unsupported operation'),
    ({'status': 6}, 'invalid error status'),
    ({'status': 3, 'body': b'x'}, 'invalid error status'),
])
def test_malformed_current_reply_and_error_statuses(connection, reply_fields, pattern):
    gateway, client = connection

    async def reply(transaction, channel):
        client.respond(transaction, **{'channel': channel, **reply_fields})

    client.raw_reply = reply
    with pytest.raises(BleGatewayError, match=pattern):
        gateway.exchange(0, b'\x01\x36\x6b')
    assert gateway.stop_all() == 5
    assert [row[3] for row in client.writes] == [4, 1, 3]


def test_partial_response_times_out_without_retry(connection):
    gateway, client = connection

    async def reply(transaction, channel):
        client.respond(transaction, channel, b'partial', more=1)

    client.raw_reply = reply
    with pytest.raises(BleGatewayError, match='timed out'):
        gateway.exchange(0, b'\x01\xfd\x6b')
    assert gateway.stop_all() == 5
    assert [row[3] for row in client.writes] == [4, 1, 3]


@pytest.mark.parametrize('close_link, early_notification', [
    (False, False), (False, True), (True, False), (True, True),
])
@pytest.mark.parametrize('connection', [2.0], indirect=True)
def test_disconnect_or_close_fails_pending_exchange(
    connection, close_link, early_notification, monkeypatch,
):
    gateway, client = connection
    started = Event()
    errors = []
    bounded_operation = gateway._with_timeout

    async def delay_exchange_result(operation, timeout, description):
        try:
            return await bounded_operation(operation, timeout, description)
        except BleGatewayError:
            # Simulate the exchange result arriving after disconnect has completed.
            if close_link and description.startswith('exchange'):
                await asyncio.sleep(0.02)
            raise

    monkeypatch.setattr(gateway, '_with_timeout', delay_exchange_result)

    async def blocked_write(transaction, channel):
        if early_notification:
            client.respond(transaction, channel, b'\x01\x36\x6b')
        started.set()
        await asyncio.Event().wait()

    def request():
        try:
            gateway.exchange(0, b'\x01\x36\x6b')
        except BleGatewayError as error:
            errors.append(str(error))

    client.raw_reply = blocked_write
    caller = Thread(target=request, daemon=True)
    caller.start()
    assert started.wait(1.0)
    if close_link:
        gateway.close()
    else:
        client.loop.call_soon_threadsafe(client.simulate_disconnect)
    caller.join(1.0)
    expected = 'closed' if close_link else 'disconnected'
    assert not caller.is_alive() and len(errors) == 1 and expected in errors[0]
    with pytest.raises(BleGatewayError, match=expected):
        gateway.exchange(0, b'\x01\x36\x6b')
    assert [row[3] for row in client.writes] == [4, 1]
    assert client.connect_calls == 1


@pytest.mark.parametrize('property_name, value, pattern', [
    ('connect_hangs', True, 'connection setup timed out'),
    ('notify_hangs', True, 'connection setup timed out'),
    ('has_service', False, 'service .* is missing'),
    ('hello', b'\x01\x02', 'Unsupported BLE gateway HELLO'),
])
def test_connection_setup_is_bounded_and_cleans_up(property_name, value, pattern):
    client = _Client()
    setattr(client, property_name, value)
    with pytest.raises(BleGatewayError, match=pattern):
        BleGateway(_MAC, 0.05, 0.08, _client_factory=client.factory)
    assert client.connect_calls == client.close_calls == 1
    assert not client.is_connected


def test_reject_invalid_control_and_local_inputs(connection):
    gateway, client = connection
    client.control_replies[3] = b'\x08'
    client.control_replies[5] = b'unexpected'
    with pytest.raises(BleGatewayError, match='invalid ACK mask'):
        gateway.stop_all()
    with pytest.raises(BleGatewayError, match='unexpected data'):
        gateway.keepalive()
    with pytest.raises(BleGatewayError, match='three valid motor addresses'):
        gateway.arm((1, 2))
    with pytest.raises(BleGatewayError, match='Invalid.*channel'):
        gateway.exchange(3, b'x')
    with pytest.raises(BleGatewayError, match='1 to 13 bytes'):
        gateway.exchange(0, bytes(14))
    assert [row[3] for row in client.writes] == [4, 3, 5]
    with pytest.raises(OSError, match='concrete Bluetooth MAC'):
        BleGateway('unconfigured', 0.2, 0.2, _client_factory=client.factory)
