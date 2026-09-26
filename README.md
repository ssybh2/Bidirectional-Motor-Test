# Bidirectional Motor Test (ROS 2 Humble)

用于 **Ubuntu 22.04 + ROS 2 Humble** 的单电机双向 DSHOT600 测试节点。

本仓库配合 [AIMEtherCAT/EcatV2_Master](https://github.com/AIMEtherCAT/EcatV2_Master) 使用，并已经按当前 EtherCAT 从站 **sn2555957** 的实际话题配置好：

- DJIRC 输入：`/ecat/sn2555957/app1/read`
- DSHOT600 输出：`/ecat/sn2555957/app2/write`
- 输入消息：`custom_msgs/msg/ReadDJIRC`
- 输出消息：`custom_msgs/msg/WriteDSHOT`
- 默认控制电机：DSHOT channel 1

## 遥控器逻辑

按 EcatV2_Master 的 DJI RC 定义，`right_switch` 为：

| right_switch | 遥控器位置 | 本节点行为 |
|---:|---|---|
| 2 | 下 | **DISARM**，发送 DSHOT = 0 |
| 3 | 中 | **正转怠速** |
| 1 | 上 | **反转怠速** |

BLHeli/BLHeli_S/BLHeli_32 的 ESC 必须已经配置为 **3D / reversible motor mode**。注意：如果只开启了 Bidirectional DShot telemetry，它只代表双向遥测，不代表电机可以反转。

直接 DSHOT 3D 模式的两个油门区间为：

- `48 ... 1047`：一个方向
- `1048 ... 2047`：另一个方向
- `0`：停止 / disarm

默认把高区间当作“正转”，低区间当作“反转”。如果你的实际电机方向相反，可把参数 `invert_direction` 设为 `true`。

> **第一次测试必须拆桨。** 电机正转时直接切反转会产生很大的机械和电气冲击。本节点在正反转切换时会先发送 0，并强制等待一段时间，再进入另一方向。

## 与当前 EtherCAT config.yaml 的对应关系

当前 EtherCAT 配置：

```yaml
slaves:
  - sn2555957:
      sdo_len: !uint16_t 7
      task_count: !uint8_t 2
      latency_pub_topic: !std::string '/ecat/sn2555957/latency'

      tasks:
        - app_1:
            sdowrite_task_type: !uint8_t 1
            conf_connection_lost_read_action: !uint8_t 0x01
            pub_topic: !std::string '/ecat/sn2555957/app1/read'
            pdoread_offset: !uint16_t 0

        - app_2:
            sdowrite_task_type: !uint8_t 4
            sdowrite_connection_lost_write_action: !uint8_t 0x01
            sub_topic: !std::string '/ecat/sn2555957/app2/write'
            pdowrite_offset: !uint16_t 0
            sdowrite_dshot_id: !uint8_t 1
            sdowrite_init_value: !uint16_t 0
```

所以本节点直接完成：

```text
/ecat/sn2555957/app1/read
        |
        v
ReadDJIRC
        |
        v
bidirectional_motor_test
        |
        v
WriteDSHOT
        |
        v
/ecat/sn2555957/app2/write
```

### EtherCAT 断线保护建议

H750 从站定义中：

- `0x01` = Keep Last
- `0x02` = Reset to Default

你当前 DSHOT app_2 使用：

```yaml
sdowrite_connection_lost_write_action: !uint8_t 0x01
```

这会在 EtherCAT 断线时保持最后一个 DSHOT 值。做推力台测试时建议改成：

```yaml
sdowrite_connection_lost_write_action: !uint8_t 0x02
sdowrite_init_value: !uint16_t 0
```

这样 EtherCAT 断线时会回到 0。

## 安全保护

节点默认启用：

- 启动后必须先把右侧三段开关拨到 **2（下，DISARM）** 一次，之后才允许电机转动。
- `DJIRC.online != 1` 时立即输出 0。
- 超过 `rc_timeout_sec` 没收到遥控器消息时立即输出 0，并重新要求一次 DISARM 解锁。
- 正转与反转之间强制插入 `direction_change_pause_sec` 的 0 输出。
- 无效的 `right_switch` 值一律输出 0。
- 除指定 `motor_channel` 外，其余 DSHOT 通道始终发送 0。
- 默认只输出较低的测试转速，不会直接给满油门。

## 默认 DSHOT 值

为了第一次台架联调，默认值设置得比较保守：

- 正转怠速：`1100`
- 正转最大：`1300`
- 反转怠速：`100`
- 反转最大：`300`

默认：

```yaml
use_throttle_axis: false
```

因此右开关拨到中/上后只使用对应怠速值。

确认拆桨测试正常后，如果希望使用左摇杆上下调速，可修改：

```yaml
use_throttle_axis: true
throttle_axis: "left_y"
```

此时摇杆中位及以下保持怠速，向上推则从怠速逐渐增加到对应方向的最大测试值。

## 你当前工作区的更新方式

你的仓库位置：

```text
/home/hby/bidirectional/Bidirectional-Motor-Test
```

拉取最新代码：

```bash
cd ~/bidirectional/Bidirectional-Motor-Test
git pull origin main
```

然后从工作区根目录重新编译：

```bash
cd ~/bidirectional

source /opt/ros/humble/setup.bash
colcon build --symlink-install

source ~/bidirectional/install/setup.bash
```

如果 `custom_msgs` 来自同一个 `~/bidirectional` 工作区中的 EcatV2_Master，则直接编译即可；如果它来自另一个已经编译好的工作区，要先 source 那个工作区，再编译本包。

## 启动

```bash
source /opt/ros/humble/setup.bash
source ~/bidirectional/install/setup.bash

ros2 launch bidirectional_motor_test motor_test.launch.py
```

启动后节点会使用：

```text
RC input : /ecat/sn2555957/app1/read
DSHOT out: /ecat/sn2555957/app2/write
```

## 第一次验证

先不要装桨。

检查 DJIRC：

```bash
ros2 topic echo /ecat/sn2555957/app1/read
```

确认：

```text
right_switch = 2  -> 下
right_switch = 3  -> 中
right_switch = 1  -> 上
```

再观察 DSHOT：

```bash
ros2 topic echo /ecat/sn2555957/app2/write
```

预期行为：

- 开关下（2）：`channel1 = 0`
- 开关中（3）：`channel1 = 1100`
- 中切上（1）：先 `channel1 = 0` 大约 1 秒，然后 `channel1 = 100`
- 重新拨下（2）：立即回到 `channel1 = 0`

其他 channel 始终为 0。

## 参数

默认参数位于：

```text
config/motor_test.yaml
```

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `input_topic` | `/ecat/sn2555957/app1/read` | DJI 遥控输入 |
| `output_topic` | `/ecat/sn2555957/app2/write` | DSHOT600 输出 |
| `motor_channel` | `1` | 1~4 号 DSHOT 通道 |
| `forward_idle_dshot` | `1100` | 正转怠速 |
| `forward_max_dshot` | `1300` | 正转最大测试值 |
| `reverse_idle_dshot` | `100` | 反转怠速 |
| `reverse_max_dshot` | `300` | 反转最大测试值 |
| `invert_direction` | `false` | 交换正反方向区间 |
| `use_throttle_axis` | `false` | 是否启用摇杆调速 |
| `throttle_axis` | `left_y` | 调速摇杆轴 |
| `throttle_deadband` | `0.05` | 摇杆死区 |
| `direction_change_pause_sec` | `1.0` | 正反切换前的停机时间 |
| `rc_timeout_sec` | `0.25` | 遥控器消息超时 |
| `require_disarm_before_arm` | `true` | 启动/失联恢复后要求先 DISARM |
| `publish_rate_hz` | `50.0` | ROS DSHOT 命令发布频率 |

## 数据流

```text
DJI RC
  |
DR16 / DBUS
  |
EtherCAT H750
  |
/ecat/sn2555957/app1/read
  |
bidirectional_motor_test
  |
/ecat/sn2555957/app2/write
  |
EcatV2_Master
  |
EtherCAT
  |
H750 DSHOT600
  |
BLHeli ESC
  |
Motor
```

真正的 DSHOT600 波形由 EtherCAT H750 从站产生；本节点只负责遥控输入、安全状态机和 DSHOT 数值命令。
