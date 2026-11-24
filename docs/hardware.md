# 三绳机器人实机接入

USB 三串口适配器和带三路 UART 的 ESP32 BLE 网关均可分别连接 `drum_1_joint`、`drum_2_joint` 和 `drum_3_joint`。每路独立时，三个驱动可以使用相同地址。适配器的电气接口须与驱动板的对应端口匹配。接线和供电按板卡型号的手册执行。每次使用一种链路，切换前停机并核对接线。

真机启动默认使用 `/ropes_real` 命名空间，使指令、反馈和 TF 与默认仿真话题隔离。房间内有其他 ROS 设备时，可为调试终端选用独立的 `ROS_DOMAIN_ID`。

当前协议实现针对 **Emm V5 旧版自由协议 `0x6B`**，用细分脉冲数设定绝对位置，用编码器回读实际轴角。X 系列固件和 Modbus RTU 需要按各自协议核对。部分驱动板型号可切换固件，识别时查看开机版本或 `FWType`。 [Emm V5 厂商手册](https://devpress.csdn.net/xian/68945b69080e555a88d61cd3.html) · [ZDT X 系列厂商手册](https://blog.csdn.net/zhangdatou666/article/details/135709033)

## USB 串口只读探测

先为三个通道分别贴上与物理卷筒一致的编号。优先选用唯一的 `/dev/serial/by-id/` 路径，同型号适配器无法为每路提供独立 ID 时使用 `/dev/serial/by-path/`。确认这三个路径分别指向独立的串口设备，且运行 ROS 的账号对每个设备均有读写权限。Ubuntu 上串口组通常是 `dialout`。若设备属于该组且当前账号没有权限，执行 `sudo usermod -aG dialout "$USER"` 后重新登录。

```bash
ls -l /dev/serial/by-id /dev/serial/by-path
cp src/ropes_hardware/config/hardware.example.yaml hardware.local.yaml
```

将 `hardware.local.yaml` 中的三个 `port` 改为实际路径，并按照板卡手册填写地址、波特率和 `pulses_per_revolution`。默认示例使用 `115200`、`8N1`、每轴地址 `1`、每转 `3200` 个命令脉冲。驱动使用串口命令控制、固定 `0x6B` 校验和接收回执 `nReceive`。每转脉冲数取决于板卡配置，启动会检查步距和细分。

从每台旧版 Emm V5 的板卡菜单核对 `S_Vel_IS=Disable`，使串口速度指令按整数 RPM 解释。设置 `O_POT_En=Disable` 关闭上电自动回零，并用厂商工具清除已保存的上电自动运行速度参数。首次接入卷筒前，在受保护的台架上重新上电并确认三台电机保持静止。 [Emm V5 厂商手册](https://devpress.csdn.net/xian/68945b69080e555a88d61cd3.html)

在 Ubuntu 上可用 `rosdep install --from-paths src --ignore-src --rosdistro humble -r -y` 安装运行依赖，串口模块来自 `python3-serial`。

```bash
source install/setup.bash
ros2 launch ropes_bringup hardware.launch.py hardware_config_path:="$PWD/hardware.local.yaml"
```

## Bluetooth 只读探测

烧录并配对 ESP32 后，使用 Bluetooth 配置示例，填写设备 MAC 和电机通道号。烧录、配对和启动命令见 [Bluetooth 网关](bluetooth.md)。

两种配置示例均以 `allow_motion: false` 开始。启动后会读取各板的版本、配置、轴角和使能/堵转状态。查看状态和原始累计轴角，并记下三个固件字节。

```bash
ros2 topic echo /ropes_real/hardware/status
ros2 topic echo /ropes_real/hardware/motor_states
```

## 标定与首次运动

1. 用厂商工具逐路核对串口与物理卷筒对应关系，并确认电机与卷筒的实际转向和传动比。关闭厂商工具后再启动 ROS，串口需要独占。配置中的 `direction` 是电机轴方向到模型卷筒方向的符号。`gear_ratio` 是电机转数除以卷筒转数。绳的收放符号继续由 `cables.json` 定义。
2. 确定每轴允许的 `lower_rad` 和 `upper_rad`，给卷筒行程、绳长和脉冲量化保留余量。按已确认的硬件设置每个 `expected_firmware`，格式如 `0xNN` 的实际十六进制值。确认兼容后，将 `allow_motion` 改为 `true` 并重启节点。
3. 用支撑或定位工装将末端置于 `cables.json` 的 `home_m`，保持三绳张紧。默认零位为 `base_link` 下的 `(0, 0, 0.17) m`。在轴静止时调用 `reference_home`，它把当前编码器反馈记录为本次会话的三个软件零位。每次重启或通信故障恢复后重新参考。
4. 确认可承接载荷的保护装置和独立实体急停已就位，驱动已使能且无故障。调用 `arm` 打开 ROS 命令门控。`arm` 授权运动指令，驱动的硬件使能由板卡和机台控制。

```bash
ros2 service call /ropes_real/hardware/reference_home std_srvs/srv/Trigger '{}'
ros2 service call /ropes_real/hardware/arm std_srvs/srv/Trigger '{}'
```

首次运动用低速、小行程验证三个卷筒的实际收放方向。默认标定下，以下示例向下移动 3 mm 后返回零位。根据台架净空和标定行程调整航点。

```bash
ros2 action send_goal /ropes_real/follow_cartesian_path ropes_interfaces/action/FollowCartesianPath \
  '{header: {frame_id: base_link}, waypoints: [{x: 0.0, y: 0.0, z: 0.167}, {x: 0.0, y: 0.0, z: 0.17}]}' \
  --feedback
```

规划器在发送命令前检查全段是否在标定行程内。规划速度同时受软件轨迹限制、配置中的卷筒上限和驱动整数 RPM 档位约束。驱动先把三轴目标分别预装载，确认后向三个串口写启动触发。USB 模式由本机发送触发，Bluetooth 模式由 ESP32 发送触发。每次发送的目标与实测轴角之间有角度限幅。实机 Action 在每个航点等待时间戳匹配的末端和卷筒反馈均进入容差，再继续下一段或返回成功。

### 首次 USB 台架验收

1. 用只读模式确认三路均能回读固件和轴角。逐路核对设备路径、驱动地址和物理卷筒，确认反馈中的关节名与通道标识一致。
2. 参考已知物理零位并解锁后，运行小行程。确认三轴转向与运动幅度符合预期，用独立量测核对末端实际经过的位移及返回零位的误差。
3. 在保持低速和小行程的条件下，确认上一组目标仍在执行时，三台驱动能接受下一组预装载目标和启动触发，轴角反馈随新目标更新，且通信与跟踪连续。
4. 再运行一次受控小行程，在运动中调用下方的 `stop`。确认三路均回执、状态显示已解除命令门控，并观察三轴实际停稳。单独验证实体急停和断开 USB 后的机械行为。

## 状态与停止

| 接口 | 用途 |
| --- | --- |
| `/ropes_real/hardware/status` | 当前门控、参考、通信故障和停止确认状态 |
| `/ropes_real/hardware/motor_states` | 原始电机轴累计角，单位 rad，参考前即可读取 |
| `/ropes_real/joint_states` | 参考后的卷筒角和 TF 用虚拟坐标 |
| `/ropes_real/end_effector_pose` | 由三个实测轴角做正运动学得到的末端估计 |
| `/ropes_real/drum_command` | 三轴携带当前时间戳的具名卷筒角命令，正常由 Action 持续发布 |
| `/ropes_real/hardware/stop` | 关闭命令门控并请求三个已经验证的驱动停止，保持驱动使能 |

取消 Action、失去有效反馈或跟踪过程中命令超时会关断 ROS 命令门控，并向仍可通信的驱动发送停止请求。BLE 断链或续期超时时，ESP32 也会直接向三台驱动发送停止帧。用 `/ropes_real/hardware/status` 判断本机收到的停止确认。确认停止后重新执行 `arm`。通信故障恢复后，检查连接和真实位置，再重启节点并重新参考。实体急停和载荷保护应独立于通信链路。

```bash
ros2 service call /ropes_real/hardware/stop std_srvs/srv/Trigger '{}'
```

位置估计依赖预张紧、固定出线点和有效卷绕半径，台架验收时需用实测位置核对。
