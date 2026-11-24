# ESP32 Bluetooth 电机网关

Bluetooth 路径是 `ROS 2 → 本机 BlueZ/BLE → ESP32-S3 → 三路 UART → Emm V5 驱动`。每个 UART 保持独立的发送和回包，三台电机的地址可以相同。USB 直连仍可使用原有的 [USB 配置与启动流程](hardware.md)。两个后端共享回零、软限位、运动学和 `/ropes_real` 下的轨迹 Action。

## ESP32-S3 固件与接线

固件工程位于 [`firmware/ropes_ble_gateway`](../firmware/ropes_ble_gateway)，目标是 **ESP32-S3 Dev Module**，使用 Arduino ESP32 3.3.1 核心自带的 NimBLE。启用 `USB CDC On Boot`，通过板子的原生 USB 端口烧录和查看日志，UART0/1/2 留给三台电机。板型和引脚布局需与实际模组核对。 [ESP32-S3 数据手册](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf) · [Arduino ESP32 BLE 示例](https://github.com/espressif/arduino-esp32/tree/3.3.1/libraries/BLE/examples)

| 通道 | ESP32-S3 RX | ESP32-S3 TX | 对应卷筒 |
| --- | --- | --- | --- |
| `0` / UART0 | GPIO4 | GPIO5 | `drum_1_joint` |
| `1` / UART1 | GPIO6 | GPIO7 | `drum_2_joint` |
| `2` / UART2 | GPIO15 | GPIO16 | `drum_3_joint` |

默认三个地址为 `1`，串口为 `115200`、`8N1`。引脚和驱动地址在 [`gateway_config.h`](../firmware/ropes_ble_gateway/gateway_config.h) 配置，需与 ROS YAML 中的 `channel`、`address` 对应。按驱动板的电气接口选用匹配的 3.3 V UART、电平转换器或收发器。RS485 需要适合 UART 的半双工自动收发切换，RS232 需要匹配的电平转换。UART 接线时交叉 RX 和 TX，并按设备手册接参考地。电机供电按驱动板要求单独接入。

在固件目录创建 `gateway_config.local.h`，在其中定义宏 `ROPES_BLE_PASSKEY` 为自己设定的六位数字，首位用 1–9。这个本地文件已加入忽略规则。默认值 `0` 时固件保持蓝牙关闭。网关启用经过认证的 BLE 加密配对，接受单个绑定主机。使用 Arduino IDE 打开 `ropes_ble_gateway.ino`，设定板型后编译并烧录，也可用 Arduino CLI 构建。

```bash
arduino-cli core install esp32:esp32@3.3.1 \
  --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc,USBMode=hwcdc \
  firmware/ropes_ble_gateway
```

## 本机配对与 ROS 启动

刷写后打开 Linux 蓝牙，在 `bluetoothctl` 中执行 `power on`、`agent KeyboardDisplay`、`default-agent` 和 `scan on`。找到名为 `ropes-ble-gateway` 的设备并记下地址，执行 `pair <MAC>`，按提示输入固件设置的六位配对码。配对后可执行 `trust <MAC>`、`scan off` 和 `quit`。使用同一绑定主机运行 ROS。 [BlueZ 设备 API](https://bluez.readthedocs.io/en/latest/device-api/)

```bash
/usr/bin/python3 -m pip install --user 'bleak==1.1.1'
cp src/ropes_hardware/config/hardware.bluetooth.example.yaml hardware.bluetooth.local.yaml
```

填写 `hardware.bluetooth.local.yaml` 中的 `bluetooth_address` 及三个驱动的通道、地址和标定值。示例以 `allow_motion: false` 运行。`transport: bluetooth` 选择 ESP32。切换回 USB 时，停机后用 `hardware.local.yaml` 启动。

```bash
source install/setup.bash
ros2 launch ropes_bringup hardware.launch.py \
  hardware_config_path:="$PWD/hardware.bluetooth.local.yaml"
```

用 `ros2 topic echo /ropes_real/hardware/status` 和 `ros2 topic echo /ropes_real/hardware/motor_states` 核对三个真实回包。只读验证完成后，按 [实机标定与首次运动](hardware.md#标定与首次运动) 确认固件版本、卷筒方向和允许行程，再在低速台架上测试回零、解锁、运动与停止。BLE 预设每秒轮询和发送轨迹命令各 5 次。

## 回包和断链停止

网关转发电机原始反馈和应答。主机用请求序号和通道关联回包，校验顺序并重组三个 UART 中的较长反馈。每次运动先确认三个暂存目标，ESP32 再从三路 UART 触发启动并返回逐轴应答位图。连接、指令或回包失效进入故障状态，重新上机需要恢复链路、确认位置并重新参考与解锁。

运动门控打开后，本机只在驱动反馈仍有效且 ROS 真机已解锁时续期。ESP32 收到断连事件或连续约 1.2 秒未续期时，关闭本地门控并向三台已配置驱动发送停止帧。应答位图可核对各路停止命令是否收到驱动确认。上机时要配合实体急停和载荷保护，并单独测试拔线与丢失蓝牙连接时的表现。

### GATT 消息

| 特征 | UUID 后缀 | 用途 |
| --- | --- | --- |
| 服务 | `4600` | 三通道网关 |
| RX | `4601` | 中央设备写请求 |
| TX | `4602` | 通知回包 |

UUID 公共前缀为 `988ebf10-90b1-4b92-9f35-1322d0bd`。每个 GATT 包最多 20 字节。请求头依次为版本 `1`、两字节小端序号、操作、通道和载荷长度。通知头包含同一版本与序号、状态、通道、原始回包偏移和是否有后续片。操作 `1` 转发 UART 帧，`2` 申请运动门控，`3` 停止，`4` 握手，`5` 续期，`6` 触发三轴。通道 `0–2` 对应 UART，`0xFF` 用于网关控制。
