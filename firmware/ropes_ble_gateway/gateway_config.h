#pragma once

#include <stdint.h>

// ESP32-S3 Dev Module, using the native USB connector for the diagnostic console.
// Each pin pair is written RX, TX. Match 3.3 V UART signal levels and share signal ground.
// Use a suitable transceiver for RS-232 or RS-485 motor interfaces.
struct MotorPortConfig {
  int rxPin;
  int txPin;
  uint8_t address;
};

constexpr MotorPortConfig kMotorPorts[] = {
    {4, 5, 1},    // UART0, drum_1_joint
    {6, 7, 1},    // UART1, drum_2_joint
    {15, 16, 1},  // UART2, drum_3_joint
};
constexpr uint32_t kMotorBaudrate = 115200;
constexpr uint32_t kUartTimeoutMs = 45;
constexpr uint32_t kLeaseMs = 1200;
constexpr char kGatewayName[] = "ropes-ble-gateway";

// Replace zero with a private six-digit passkey before commissioning. Zero keeps BLE offline.
#if __has_include("gateway_config.local.h")
#include "gateway_config.local.h"
#endif
#ifndef ROPES_BLE_PASSKEY
#define ROPES_BLE_PASSKEY 0
#endif
constexpr uint32_t kBlePasskey = ROPES_BLE_PASSKEY;
static_assert(kBlePasskey == 0 || (kBlePasskey >= 100000 && kBlePasskey <= 999999),
              "Use a private six-digit BLE passkey");
