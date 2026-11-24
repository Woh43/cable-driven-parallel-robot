#include "gateway.h"

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLESecurity.h>
#include <host/ble_hs.h>
#include <host/ble_hs_mbuf.h>
#include <host/ble_store.h>
#include <freertos/queue.h>

#include <algorithm>
#include <cstring>

#include "gateway_config.h"

#if !defined(CONFIG_IDF_TARGET_ESP32S3) || !defined(CONFIG_NIMBLE_ENABLED)
#error This gateway requires the ESP32-S3 and bundled Arduino NimBLE BLE backend.
#endif
#if !defined(CONFIG_BT_NIMBLE_SECURITY_ENABLE)
#error Enable BLE Security Manager support to use the protected GATT service.
#endif
#if !ARDUINO_USB_CDC_ON_BOOT
#error Enable USB CDC On Boot so diagnostic logging uses native USB, not motor UART0.
#endif

namespace {
constexpr char kService[] = "988ebf10-90b1-4b92-9f35-1322d0bd4600";
constexpr char kRx[] = "988ebf10-90b1-4b92-9f35-1322d0bd4601";
constexpr char kTx[] = "988ebf10-90b1-4b92-9f35-1322d0bd4602";
constexpr uint8_t kVersion = 1;
constexpr uint8_t kControl = 0xFF;
constexpr uint8_t kAxisCount = 3;
constexpr uint8_t kAllAxes = 0x07;
constexpr uint8_t kFrameEnd = 0x6B;
constexpr uint8_t kAck = 0x02;
constexpr uint8_t kMove = 0xFD;
constexpr uint8_t kStop = 0xFE;
constexpr uint8_t kSync = 0xFF;
constexpr size_t kPacketSize = 20;
constexpr size_t kRequestHeader = 6;
constexpr size_t kReplyHeader = 7;
constexpr size_t kUartMaxReply = 33;

static_assert(sizeof(kMotorPorts) / sizeof(kMotorPorts[0]) == kAxisCount);
static_assert(kMotorPorts[0].address && kMotorPorts[1].address && kMotorPorts[2].address,
              "Configure unicast Emm V5 addresses on all channels");

HardwareSerial *const kUarts[] = {&Serial0, &Serial1, &Serial2};
BLEServer *server = nullptr;
BLECharacteristic *tx = nullptr;
QueueHandle_t requests = nullptr;
portMUX_TYPE peerMux = portMUX_INITIALIZER_UNLOCKED;

struct Peer {
  uint32_t generation = 0;
  uint16_t id = 0;
  bool connected = false;
  bool authenticated = false;
  bool notificationsEnabled = false;
};

struct Request {
  uint32_t generation;
  uint16_t connection;
  uint8_t size;
  uint8_t data[kPacketSize];
};

struct UartReply {
  uint8_t bytes[kUartMaxReply];
  uint8_t size = 0;
  uint8_t expected = 0;
  bool failed = false;
};

struct GatewayState {
  bool armed = false;
  uint32_t generation = 0;
  uint32_t lastRenewal = 0;
  uint8_t staged = 0;
};

Peer peer;
ble_addr_t trustedAddress{};
bool hasTrustedAddress = false;
GatewayState state;

enum class Op : uint8_t { Transfer = 1, Arm, StopAll, Hello, Keepalive, StartGroup };
enum class ReplyStatus : uint8_t { Ok, Malformed, NotArmed, UartError, Busy, Unsupported };

Peer peerSnapshot() {
  portENTER_CRITICAL(&peerMux);
  const Peer snapshot = peer;
  portEXIT_CRITICAL(&peerMux);
  return snapshot;
}

bool activeSession(uint32_t generation, uint16_t connection) {
  const Peer snapshot = peerSnapshot();
  return snapshot.connected && snapshot.authenticated && snapshot.notificationsEnabled &&
         snapshot.generation == generation && snapshot.id == connection;
}

bool leaseActive() {
  const Peer snapshot = peerSnapshot();
  return state.armed && snapshot.connected && snapshot.authenticated &&
         snapshot.notificationsEnabled &&
         snapshot.generation == state.generation &&
         uint32_t(millis() - state.lastRenewal) < kLeaseMs;
}

bool sendReply(const Request &request, ReplyStatus status, const uint8_t *body = nullptr,
               size_t bodySize = 0) {
  size_t offset = 0;
  do {
    if (!activeSession(request.generation, request.connection)) {
      return false;
    }
    const size_t count = std::min(kPacketSize - kReplyHeader, bodySize - offset);
    uint8_t packet[kPacketSize] = {
        kVersion, request.data[1], request.data[2], static_cast<uint8_t>(status),
        request.data[4], static_cast<uint8_t>(offset), uint8_t(offset + count < bodySize),
    };
    if (count) {
      std::memcpy(packet + kReplyHeader, body + offset, count);
    }
    auto *buffer = ble_hs_mbuf_from_flat(packet, kReplyHeader + count);
    // NimBLE consumes the allocation regardless of the return code.
    if (buffer == nullptr || ble_gatts_notify_custom(request.connection, tx->getHandle(), buffer)) {
      return false;
    }
    offset += count;
  } while (offset < bodySize);
  return true;
}

bool sameAddress(const ble_addr_t &left, const ble_addr_t &right) {
  // Public and random identity types can also carry the controller's resolved bit.
  return (left.type & 1) == (right.type & 1) && std::memcmp(left.val, right.val, 6) == 0;
}

class PeerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s, ble_gap_conn_desc *incoming) override {
    portENTER_CRITICAL(&peerMux);
    const bool allowed = !peer.connected && (!hasTrustedAddress ||
        sameAddress(trustedAddress, incoming->peer_id_addr));
    if (allowed) {
      peer.id = incoming->conn_handle;
      ++peer.generation;
      peer.connected = true;
      peer.authenticated = false;
      peer.notificationsEnabled = false;
    }
    portEXIT_CRITICAL(&peerMux);
    if (!allowed) {
      s->disconnect(incoming->conn_handle);
      return;
    }
    const int rc = ble_gap_security_initiate(incoming->conn_handle);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
      s->disconnect(incoming->conn_handle);
    }
  }

  void onDisconnect(BLEServer *, ble_gap_conn_desc *incoming) override {
    portENTER_CRITICAL(&peerMux);
    if (peer.connected && peer.id == incoming->conn_handle) {
      peer.connected = false;
      peer.authenticated = false;
      peer.notificationsEnabled = false;
      ++peer.generation;
    }
    const bool advertise = !peer.connected;
    portEXIT_CRITICAL(&peerMux);
    if (advertise) {
      BLEDevice::startAdvertising();
    }
  }
};

class PairingCallbacks : public BLESecurityCallbacks {
  bool onConfirmPIN(uint32_t) override {
    return false;  // Require entry of the configured passkey by the central.
  }

  void onAuthenticationComplete(ble_gap_conn_desc *result) override {
    portENTER_CRITICAL(&peerMux);
    const bool current = peer.connected && peer.id == result->conn_handle;
    const bool authorized = result->sec_state.encrypted && result->sec_state.authenticated &&
        result->sec_state.bonded &&
        (!hasTrustedAddress || sameAddress(trustedAddress, result->peer_id_addr));
    if (current) {
      peer.authenticated = authorized;
      if (authorized && !hasTrustedAddress) {
        trustedAddress = result->peer_id_addr;
        hasTrustedAddress = true;
      }
    }
    portEXIT_CRITICAL(&peerMux);
    if (current && !authorized) {
      server->disconnect(result->conn_handle);
    }
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onSubscribe(BLECharacteristic *, ble_gap_conn_desc *incoming, uint16_t value) override {
    portENTER_CRITICAL(&peerMux);
    if (peer.connected && peer.id == incoming->conn_handle) {
      const bool enabled = (value & 0x01) != 0;
      if (peer.notificationsEnabled && !enabled) {
        ++peer.generation;
      }
      peer.notificationsEnabled = enabled;
    }
    portEXIT_CRITICAL(&peerMux);
  }

  void onWrite(BLECharacteristic *characteristic, ble_gap_conn_desc *incoming) override {
    const Peer snapshot = peerSnapshot();
    const size_t length = characteristic->getLength();
    if (!snapshot.connected || !snapshot.authenticated || incoming->conn_handle != snapshot.id ||
        !snapshot.notificationsEnabled || length < kRequestHeader) {
      return;
    }
    Request request{snapshot.generation, snapshot.id, 0, {}};
    // The GATT attribute also requires authenticated encryption.
    const uint8_t *bytes = characteristic->getData();
    std::memcpy(request.data, bytes, kRequestHeader);
    if (length > kPacketSize) {
      sendReply(request, ReplyStatus::Malformed);
      return;
    }
    request.size = length;
    std::memcpy(request.data + kRequestHeader, bytes + kRequestHeader, length - kRequestHeader);
    if (xQueueSend(requests, &request, 0) != pdTRUE) {
      sendReply(request, ReplyStatus::Busy);
    }
  }
};

PeerCallbacks peerCallbacks;
PairingCallbacks pairingCallbacks;
CommandCallbacks commandCallbacks;

void discardUartInput(uint8_t channel) {
  // Bound the drain even if a miswired or misconfigured drive continuously emits bytes.
  for (size_t read = 0; read < 256 && kUarts[channel]->read() >= 0; ++read) {}
}

void receiveUart(uint8_t channel, uint8_t code, UartReply &reply) {
  const uint8_t address = kMotorPorts[channel].address;
  while (!reply.failed && reply.size < reply.expected) {
    const int byte = kUarts[channel]->read();
    if (byte < 0) {
      break;
    }
    reply.bytes[reply.size++] = static_cast<uint8_t>(byte);
    if (reply.size == 1 && reply.bytes[0] != address) {
      reply.failed = true;
    } else if (reply.size == 2 && reply.bytes[1] != code) {
      reply.failed = reply.bytes[1] != 0x00;
      reply.expected = 4;
    }
  }
  if (reply.size == reply.expected && reply.bytes[reply.size - 1] != kFrameEnd) {
    reply.failed = true;
  }
}

bool uartFrame(uint8_t channel, const uint8_t *frame, size_t length, UartReply &reply,
               bool requireLease) {
  discardUartInput(channel);
  if ((requireLease && !leaseActive()) || kUarts[channel]->write(frame, length) != length) {
    return false;
  }
  const uint32_t start = millis();
  do {
    receiveUart(channel, frame[1], reply);
    if (reply.failed || reply.size == reply.expected) {
      return !reply.failed;
    }
    delay(1);
  } while (uint32_t(millis() - start) < kUartTimeoutMs && (!requireLease || leaseActive()));
  return false;
}

uint8_t groupCommand(uint8_t code, uint8_t key, bool requireLease) {
  UartReply replies[kAxisCount];
  uint8_t pending = 0;
  for (uint8_t channel = 0; channel < kAxisCount; ++channel) {
    discardUartInput(channel);
    if (requireLease && !leaseActive()) {
      break;
    }
    uint8_t frame[] = {kMotorPorts[channel].address, code, key, 0, kFrameEnd};
    const size_t length = code == kSync ? 4 : 5;
    if (code == kSync) {
      frame[3] = kFrameEnd;
    }
    if (kUarts[channel]->write(frame, length) != length) {
      if (requireLease) {
        break;
      }
      continue;
    }
    replies[channel].expected = 4;
    pending |= uint8_t(1 << channel);
  }
  if (requireLease && pending != kAllAxes) {
    return 0;
  }

  uint8_t completed = 0;
  const uint32_t start = millis();
  while (pending && uint32_t(millis() - start) < kUartTimeoutMs &&
         (!requireLease || leaseActive())) {
    for (uint8_t channel = 0; channel < kAxisCount; ++channel) {
      const uint8_t bit = uint8_t(1 << channel);
      if (!(pending & bit)) {
        continue;
      }
      UartReply &reply = replies[channel];
      receiveUart(channel, code, reply);
      if (reply.failed || reply.size == reply.expected) {
        pending &= ~bit;
        if (!reply.failed && reply.bytes[1] == code && reply.bytes[2] == kAck) {
          completed |= bit;
        }
      }
    }
    if (pending) {
      delay(1);
    }
  }
  return completed;
}

uint8_t stopMotors() {
  state.armed = false;
  state.staged = 0;
  return groupCommand(kStop, 0x98, false);
}

bool canArm(const Request &request, ReplyStatus &status) {
  for (uint8_t channel = 0; channel < kAxisCount; ++channel) {
    if (request.data[kRequestHeader + channel] != kMotorPorts[channel].address) {
      status = ReplyStatus::Unsupported;
      return false;
    }
  }
  for (uint8_t channel = 0; channel < kAxisCount; ++channel) {
    const uint8_t query[] = {kMotorPorts[channel].address, 0x3A, kFrameEnd};
    UartReply reply;
    reply.expected = 4;
    if (!activeSession(request.generation, request.connection) ||
        !uartFrame(channel, query, sizeof(query), reply, false) || reply.bytes[1] != 0x3A ||
        (reply.bytes[2] & 0x0D) != 0x01) {
      status = ReplyStatus::UartError;
      return false;
    }
  }
  return activeSession(request.generation, request.connection);
}

ReplyStatus transfer(const Request &request, UartReply &reply) {
  const uint8_t channel = request.data[4];
  const size_t size = request.data[5];
  const uint8_t *frame = request.data + kRequestHeader;
  if (channel >= kAxisCount || size < 3 || frame[0] != kMotorPorts[channel].address ||
      frame[size - 1] != kFrameEnd) {
    return ReplyStatus::Malformed;
  }
  const uint8_t code = frame[1];
  bool valid = false;
  switch (code) {
    case 0x1F: reply.expected = 5; valid = size == 3; break;
    case 0x36: reply.expected = 8; valid = size == 3; break;
    case 0x3A: reply.expected = 4; valid = size == 3; break;
    case 0x42: reply.expected = 33; valid = size == 4 && frame[2] == 0x6C; break;
    case kMove:
      reply.expected = 4;
      if (size == 13) {
        const uint16_t speed = uint16_t(frame[3]) << 8 | frame[4];
        valid = frame[2] <= 1 && speed >= 1 && speed <= 5000 &&
                frame[10] == 1 && frame[11] == 1;
      }
      break;
    default: return ReplyStatus::Unsupported;
  }
  if (!valid) {
    return ReplyStatus::Malformed;
  }
  if (code == kMove && !leaseActive()) {
    return ReplyStatus::NotArmed;
  }
  if (!uartFrame(channel, frame, size, reply, code == kMove)) {
    if (state.armed) {
      stopMotors();
    }
    return ReplyStatus::UartError;
  }
  if (code == kMove) {
    if (reply.bytes[1] == kMove && reply.bytes[2] == kAck) {
      state.staged |= uint8_t(1 << channel);
    } else {
      // Preserve a vendor error frame for host diagnostics, but discard the entire move.
      stopMotors();
    }
  }
  return ReplyStatus::Ok;
}

void handleRequest(const Request &request) {
  if (!activeSession(request.generation, request.connection)) {
    return;
  }
  const uint8_t *data = request.data;
  const auto op = static_cast<Op>(data[3]);
  const uint8_t size = data[5];
  if (data[0] != kVersion || request.size != kRequestHeader + size ||
      (op != Op::Transfer && data[4] != kControl)) {
    sendReply(request, ReplyStatus::Malformed);
    return;
  }
  if (op != Op::Transfer && size != (op == Op::Arm ? kAxisCount : 0)) {
    sendReply(request, ReplyStatus::Malformed);
    return;
  }

  UartReply uart;
  ReplyStatus status = ReplyStatus::Ok;
  uint8_t small[]{kVersion, kAxisCount};
  const uint8_t *body = small;
  size_t replySize = 0;
  switch (op) {
    case Op::Hello:
      replySize = 2;
      break;
    case Op::Transfer:
      status = transfer(request, uart);
      if (status == ReplyStatus::Ok) {
        body = uart.bytes;
        replySize = uart.size;
      }
      break;
    case Op::Arm:
      if (state.armed) {
        status = ReplyStatus::Busy;
      } else if (canArm(request, status)) {
        state = GatewayState{true, request.generation, millis(), 0};
      }
      break;
    case Op::StopAll:
      small[0] = stopMotors();
      replySize = 1;
      break;
    case Op::Keepalive:
      if (leaseActive()) {
        state.lastRenewal = millis();
      } else {
        status = ReplyStatus::NotArmed;
      }
      break;
    case Op::StartGroup:
      if (!leaseActive()) {
        status = ReplyStatus::NotArmed;
      } else if (state.staged != kAllAxes) {
        status = ReplyStatus::Malformed;
        stopMotors();
      } else {
        state.staged = 0;
        small[0] = groupCommand(kSync, 0x66, true);
        if (small[0] != kAllAxes) {
          replySize = 1;  // The host uses the confirmed start mask to report a partial start.
          stopMotors();
        } else if (!leaseActive()) {
          status = ReplyStatus::NotArmed;
          stopMotors();
        } else {
          replySize = 1;
        }
      }
      break;
    default:
      status = ReplyStatus::Unsupported;
      break;
  }
  if (!sendReply(request, status, body, replySize) && state.armed) {
    stopMotors();
  }
}

bool loadBond() {
  ble_addr_t addresses[2]{};
  int count = 0;
  if (ble_store_util_bonded_peers(addresses, &count, 2) != 0 || count > 1) {
    return false;
  }
  if (count == 1) {
    trustedAddress = addresses[0];
    hasTrustedAddress = true;
  }
  return true;
}

void configureSecurity() {
  // Configure the SDK host directly so only the admitted peer starts pairing.
  ble_hs_cfg.sm_bonding = 1;
  ble_hs_cfg.sm_mitm = 1;
  ble_hs_cfg.sm_sc = 1;
  ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_DISP_ONLY;
  ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
  ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
  BLESecurity::setPassKey(true, kBlePasskey);
}
}  // namespace

void setupGateway() {
  Serial.begin(115200);
  if (kBlePasskey == 0) {
    Serial.println("BLE gateway is disabled until a private six-digit passkey is configured.");
    return;
  }
  requests = xQueueCreate(1, sizeof(Request));
  if (requests == nullptr) {
    Serial.println("Could not create the BLE command queue.");
    return;
  }
  for (uint8_t channel = 0; channel < kAxisCount; ++channel) {
    const auto &port = kMotorPorts[channel];
    kUarts[channel]->begin(kMotorBaudrate, SERIAL_8N1, port.rxPin, port.txPin);
  }

  BLEDevice::init(kGatewayName);
  if (!loadBond()) {
    Serial.println("Expected a single bonded central. Clear controller bonds before commissioning.");
    return;
  }
  configureSecurity();
  BLEDevice::setSecurityCallbacks(&pairingCallbacks);

  server = BLEDevice::createServer();
  server->setCallbacks(&peerCallbacks);
  BLEService *service = server->createService(kService);
  auto *rx = service->createCharacteristic(kRx, BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_WRITE_ENC | BLECharacteristic::PROPERTY_WRITE_AUTHEN);
  rx->setCallbacks(&commandCallbacks);
  tx = service->createCharacteristic(kTx, BLECharacteristic::PROPERTY_NOTIFY);
  tx->setCallbacks(&commandCallbacks);
  service->start();
  BLEDevice::getAdvertising()->addServiceUUID(kService);
  BLEDevice::startAdvertising();
  Serial.println("BLE UART gateway ready. Pair the authorized Linux central before use.");
}

void runGateway() {
  if (requests == nullptr || server == nullptr) {
    delay(100);
    return;
  }
  Request request{};
  const bool received = xQueueReceive(requests, &request, pdMS_TO_TICKS(5)) == pdTRUE;
  if (state.armed && !leaseActive()) {
    const uint8_t stopped = stopMotors();
    Serial.printf("BLE lease expired or disconnected; stop ACK mask 0x%02x\n", stopped);
  }
  if (received) {
    handleRequest(request);
  }
}
