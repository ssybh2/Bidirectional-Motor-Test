# Bidirectional Motor Test (ROS 2 Humble)

用于 **Ubuntu 22.04 + ROS 2 Humble** 的单电机双向 DSHOT600 测试节点。

本仓库配合 [AIMEtherCAT/EcatV2_Master](https://github.com/AIMEtherCAT/EcatV2_Master) 使用：

- 输入话题：`/DJIRC`
- 输出话题：`/dshot600`
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

BLHeli/BLHeli_S/BLHeli_32 的 ESC 必须已经配置为 bidirectional / 3D 模式。

直接 DSHOT 3D 模式的两个油门区间为：

- `48 ... 1047`：一个方向
- `1048 ... 2047`：另一个方向
- `0`：停止 / disarm

默认把高区间当作“正转”，低区间当作“反转”。如果你的实际电机方向相反，可把参数 `invert_direction` 设为 `true`。

> **第一次测试必须拆桨。** 电机正转时直接切反转会产生很大的机械和电气冲击。本节点在正反转切换时会先发送 0，并强制等待一段时间，再进入另一方向。

## 安全保护

节点默认启用以下保护：

- 启动后必须先把右侧三段开关拨到 **2（下，DISARM）** 一次，之后才允许电机转动。
- `DJIRC.online != 1` 时立即输出 0。
- 超过 `rc_timeout_sec` 没收到遥控器消息时立即输出 0，并重新要求一次 DISARM 解锁。
- 正转与反转之间强制插入 `direction_change_pause_sec` 的 0 输出。
- 无效的 `right_switch` 值一律输出 0。
- 除指定 `motor_channel` 外，其余 DSHOT 通道始终发送 0。
- 默认只输出较低的测试转速，不会直接给满油门。

## 默认 DSHOT 值

默认参数为了台架初次联调而故意设置得比较保守：

- 正转怠速：`1100`
- 正转最大：`1300`
- 反转怠速：`100`
- 反转最大：`300`

默认 `use_throttle_axis: false`，因此右开关拨到中/上后只会使用对应的怠速值。

如果确认拆桨测试正常后，希望用遥控器左摇杆上下控制转速，可在配置中改成：

```yaml
use_throttle_axis: true
throttle_axis: left_y
```

此时：

- 摇杆中位及以下：怠速
- 向上推：从怠速逐渐增加到配置的最大 DSHOT 值

## EtherCAT 端配置

在 EcatV2_Master 的配置生成器中至少创建两个 task：

1. **DJI RC**
   - Publisher Topic Name: `/DJIRC`

2. **DSHOT600**
   - Motor Command Subscriber Topic Name: `/dshot600`
   - Initial Value: **0**
   - Connection Lost Action: **Reset to Default**
   - TIM/DSHOT 端口选择你实际连接 ESC 信号线的端口

这里特别建议 DSHOT 的初始值使用 **0**，不要在电机测试台上使用非零初始值。

## 工作区安装

假设你的 ROS 2 工作区是 `~/one`：

```bash
cd ~/one/src
git clone https://github.com/ssybh2/Bidirectional-Motor-Test.git

# EcatV2_Master 必须也位于同一个 colcon 工作区中，
# 因为本包依赖它提供的 custom_msgs。
cd ~/one
source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

source install/setup.bash
```

## 启动

```bash
source /opt/ros/humble/setup.bash
source ~/one/install/setup.bash

ros2 launch bidirectional_motor_test motor_test.launch.py
```

观察遥控器输入：

```bash
ros2 topic echo /DJIRC
```

观察本节点发给 EtherCAT 的 DSHOT：

```bash
ros2 topic echo /dshot600
```

正常情况下，你应看到：

- 右开关下：`channel1: 0`
- 右开关中：`channel1: 1100`（默认）
- 中切到上：先保持 `channel1: 0` 约 1 秒，然后变为 `channel1: 100`
- 右开关再次下：立刻回到 `channel1: 0`

## 参数

默认参数位于：

```text
config/motor_test.yaml
```

主要参数：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `input_topic` | `/DJIRC` | DJI 遥控输入 |
| `output_topic` | `/dshot600` | DSHOT600 输出 |
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
| `require_disarm_before_arm` | `true` | 启动/失联恢复后是否要求先 DISARM |
| `publish_rate_hz` | `50.0` | DSHOT ROS 消息发布频率 |

## 重要说明

这个节点只负责：

```text
DJIRC -> 安全状态机 -> WriteDSHOT -> /dshot600
```

真正的 DSHOT600 波形由 EtherCAT H750 从站产生。

请先在**无桨**状态确认：

1. 右开关下时始终是 0。
2. 右开关中时电机按预期方向低速旋转。
3. 中切上时，电机先停下，再反向低速旋转。
4. 遥控器关机/接收机掉线时电机立即停止。
5. EtherCAT 断线时从站配置确实会回到 Initial Value = 0。

确认这些都正确后再进入推力测试台测试。
