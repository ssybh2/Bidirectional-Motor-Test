# Bidirectional Motor Test · 正弦换向与推力延迟记录

运行环境：**Ubuntu 22.04 / ROS 2 Humble**。本包是 [AIMEtherCAT/EcatV2_Master](https://github.com/AIMEtherCAT/EcatV2_Master) 的上层 ROS 控制节点，不负责产生物理 DSHOT600 波形。

## 实际连接的话题

| 数据 | 话题 | ROS 消息 |
|---|---|---|
| DJI RC 输入 | `/ecat/sn2555957/app1/read` | `custom_msgs/msg/ReadDJIRC` |
| DSHOT 输出 | `/ecat/sn2555957/app2/write` | `custom_msgs/msg/WriteDSHOT` |
| 可选推力数据 | 默认**未连接**；由用户指定 `force_topic` | `std_msgs/msg/Float64` |
| 可选转速数据 | 默认**未连接**；由用户指定 `rpm_topic` | `std_msgs/msg/Float64` |

默认 DSHOT 输出为 `channel1`；其他 channel 始终写入 0。上面两个 EtherCAT 话题已与从站 `sn2555957` 对齐。

## right_switch 行为

| 位置 | 数值 | 本节点行为 |
|---|---:|---|
| 下 | 2 | **DISARM**，连续发送 DSHOT 0，并取消测试 |
| 中 | 3 | **ARMED / 待命**，仍然连续发送 DSHOT 0；不会自行旋转 |
| 上 | 1 | **SINE / 正弦换向实验**，正半周正转、负半周反转 |

启动以及遥控失联恢复后，必须按顺序走 **2 → 3 → 1**。不能直接从 2 跳到 1。1 → 3 会立刻撤掉正弦命令并输出 0；重新 3 → 1 还需要经过 `run_reentry_pause_sec` 的零输出等待，避免迅速拨动开关绕过换向等待。

## 波形如何映射到 DSHOT

定义逻辑正弦量：

```text
u = sin(2π f t)

u > 0  => DSHOT 1048 .. positive_peak_dshot  => 方向 A
u < 0  => DSHOT   48 .. negative_peak_dshot  => 方向 B
|u| <= sine_deadband => DSHOT 0
```

使用 `invert_direction: true` 可以交换方向 A/B 与“正转/反转”的对应关系；**实际物理正转是哪一边，必须拆桨确认。** 正弦量为正/负时，DSHOT 数值本身并不是正数/负数；负半周被编码到 DSHOT 的较低 3D 区间。

默认 `sine_frequency_hz: 0.05`，理论正弦周期 20 秒；`positive_peak_dshot: 1250`、`negative_peak_dshot: 250`。这些仅是有界测试值，不能保证对任何 ESC/电机都属于安全转速。

每次需要改变方向时，节点先输出 DSHOT 0，至少等待 `reversal_pause_sec: 2.0`，然后才发送相反方向的第一条非零指令。等待期间**冻结正弦相位**，以免恢复时跳到大油门。因此实际墙钟时间上的曲线包含零输出平台，**不是严格连续、不中断的数学正弦波**。

**固定等待时间不能证明电机已经停转。** 有桨实验前应接入真实 RPM ROS 话题，并设置：

```yaml
rpm_topic: "/your_real_rpm_topic"
require_rpm_for_reversal: true
rpm_stop_threshold: 100.0
rpm_stable_sec: 0.3
```

这样在最短停机时间满足后，还要连续收到新鲜、低于阈值的转速数据，才允许换向。没有转速反馈时默认可以做**拆桨的软件/电气联调**，但不能据此认为有桨换向安全。正反转还要检查螺旋桨方向、固定方式、螺纹是否可能松脱及台架防护。

## 指令到推力变化：到底量了什么

节点每个控制周期记录：

- `command.csv`：ROS 调用 publish 时的单调时钟时间、实际 DSHOT 指令、带符号的正弦量和模式。
- `force.csv`：推力 ROS 消息**抵达本节点**的单调时钟时间、数值和当时最新的 DSHOT。
- `event.csv`：解锁、正弦启动、换向等待以及**相反方向第一条非零命令**的时刻；每个测试事件分配 `event_id`。
- `latency.csv`：每个 `event_id` 的两个指标：`force_onset`（推力向目标方向变化超过阈值），`target_sign`（推力达到目标符号及阈值）。需要连续 `force_confirm_samples` 个样本满足条件；记录的是第一个满足条件的样本时间。

事件的 **t₀** 是 `first_command` / `reversal_command` 的首次非零 DSHOT 发布时刻，**不是**正弦过零、也不是开始强制等待的时刻。由此不会把人工设定的 `reversal_pause_sec` 直接算进“指令→推力”延迟。

```text
t0 = 首次反向非零 DSHOT publish（ROS 主机单调时钟）
t1 = 满足显著变化判据的第一条推力样本时刻
推力变化观测延迟 = t1 - t0

t2 = 连续达到目标符号阈值的第一条推力样本时刻
推力换向观测延迟 = t2 - t0
```

直接 G10 原始计数模式默认显著变化/符号阈值为 50 counts，连续确认 10 个原生样本（1 ms）；完成 kgf 标定后可从 0.03 kgf、3 点开始再按噪声调整。ROS 推力话题模式下，阈值单位与话题一致。否则噪声或原方向的惯性衰减可能造成“变化开始”的误判。

这两个时间差是**ROS 指令发布到推力变化的观测延迟**，包括 ROS 调度、EtherCAT、ESC、电机、推力传感器采集与传输等影响。不是纯 ESC 延迟，也不等价于硬件同步的真实力学延迟。ROS 默认发布频率 50 Hz 本身带来约 20 ms 的命令离散化；直接 G10 模式可保留 0.1 ms 的传感器样本间隔，但绝对时刻仍包含设备批处理及网络传输偏差。只有硬件触发才能进一步分解各环节。

### G10 推力数据接入条件

现在已经根据 G10X.322.0.0.492 与 DET G10-10KGF-5 的实机抓包加入直接 UDP 接收。已确认的线格式是：

```text
192.168.127.56:5000 -> 192.168.127.55:4800/UDP
986-byte payload = 10-byte header + 40 x 22-byte sample record
                     + 4 x 22-byte status record + 8-byte trailer
每个有效记录包含 8 路 signed big-endian int16 ADC；约 250 包/s，即标称每路 10 kHz。
```

在这台 SN `DET50316-62-50307-1` 上，从零开始编号的 `g10_adc_channel: 6`（线上第 7 路 ADC）对轻压推力传感器的响应远大于其他通道，暂定为推力原始通道。节点直接在同一 Ubuntu 进程内给 UDP 批次重建 100 µs 样本时刻，并与 DSHOT publish 的 `time.monotonic_ns()` 比较，因此不再需要同步 Windows 时钟。

默认配置已经启用直接接收：

```yaml
force_topic: ""
g10_udp_enabled: true
g10_adc_channel: 6
g10_auto_zero: true
g10_auto_zero_samples: 10000
g10_force_sign: -1
g10_kgf_per_count: 0.0
```

`g10_kgf_per_count: 0.0` 表示先用 `raw_count` 测量。启动前让台架完全卸载，节点用最初 1 秒的 10000 点自动归零。此模式的建议初始阈值为 50 counts、连续 10 点（1 ms）。接着放置已知质量 `M_kg` 的砝码，记录稳定后的 `raw_loaded` 和零点 `raw_zero`：

```text
g10_kgf_per_count = M_kg / abs(raw_loaded - raw_zero)
```

填入比例后，`force_unit` 自动变成 kgf，并应把变化/符号阈值按噪声改回约 0.03 kgf 起步。`g10_force_sign` 决定哪一侧为正；如果正转推力显示为负，改成 `1`。

G10 必须接 Ubuntu 的第二块独立网卡并设为 `192.168.127.55/24`；EtherCAT 继续独占原来的实时网卡。不要把两个实时协议接在同一物理口上。

若禁用直接 UDP、改用其他推力桥接，也仍可使用原来的 ROS 话题模式：

```yaml
g10_udp_enabled: false
force_topic: "/g10/thrust"
force_unit: "kgf"
force_forward_sign: 1
```

若正向转动时推力数据为负值，设置 `force_forward_sign: -1`。这只是测量符号校准，不会改变电机转向。

若只有 Windows 厂家软件的 CSV 数据、没有与 ROS 共享的时钟或同步事件，**仅凭两个各自独立的 CSV 时间戳不能可信地计算毫秒级物理延迟**。直接 G10 UDP 模式解决的是跨主机时钟问题；设备内部 ADC 滤波、UDP 批处理和 DSHOT 实际发射相对 ROS publish 的偏差仍属于测量链路延迟。要分解到纯硬件响应，仍需 GPIO/逻辑分析仪触发标记。

## 下载、编译和启动

你的目录是 `/home/hby/bidirectional/Bidirectional-Motor-Test`，请以普通 `hby` 用户执行，不要使用 `sudo su` 编译：

```bash
cd /home/hby/bidirectional/Bidirectional-Motor-Test
git pull --ff-only origin main

cd /home/hby/bidirectional
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to bidirectional_motor_test
source /home/hby/bidirectional/install/setup.bash

ros2 launch bidirectional_motor_test motor_test.launch.py
```

新开终端检查当前输出（这一步**不需要电机动力上电**）：

```bash
source /opt/ros/humble/setup.bash
source /home/hby/bidirectional/install/setup.bash
ros2 topic echo /ecat/sn2555957/app1/read
ros2 topic echo /ecat/sn2555957/app2/write
```

使用 Ctrl+C 结束，节点会尝试发送十次 DSHOT 0，但这**不是硬件急停**。日志保存在：

```text
/home/hby/bidirectional/measurements/test_<UTC时间>_<进程ID>_command.csv
/home/hby/bidirectional/measurements/test_<UTC时间>_<进程ID>_force.csv
/home/hby/bidirectional/measurements/test_<UTC时间>_<进程ID>_event.csv
/home/hby/bidirectional/measurements/test_<UTC时间>_<进程ID>_latency.csv
```

例如查看最近的测量会话：

```bash
ls -lt ~/bidirectional/measurements | head -12
```

## 关键硬件安全限制

**不要将软件 DISARM 当作断电保证。** 在当前 EcatV2_Master / H750 软件路径中，若只有本 ROS 控制节点退出、EtherCAT master 仍运行，从站可能继续接收到/保持最后的非零 DSHOT。节点的退出清零属于 best-effort，不能覆盖进程崩溃、系统卡死和线路失效。必须有独立、可立即切断 ESC 动力的物理急停。

特别核查 EtherCAT 配置中 DSHOT task 的断线动作：

```yaml
sdowrite_connection_lost_write_action: !uint8_t 0x02  # RESET_TO_DEFAULT
sdowrite_init_value: !uint16_t 0
```

原来的 `0x01` 是 Keep Last，不适合台架失联安全。但 **0x02 仅覆盖 EtherCAT 链路断线，不覆盖 ROS 发布节点单独停止**。这一点需要在 H750 端新增“无新 ROS 命令时自动清零”的独立看门狗，才可进一步降低风险。

请先拆桨检查开关逻辑、波形数值和消息连通性。接上实际螺旋桨前，先核对 ESC 真正开启的是 **3D / reversible motor mode**（不是仅启用了 Bidirectional DShot telemetry），并完成电机固定、转速联锁、物理急停、防护、量程及电源极性检查。

## 代码结构和逻辑测试

- `bidirectional_motor_test/core.py`：开关解锁状态机与正弦 DSHOT 映射，含停机保持。
- `bidirectional_motor_test/latency.py`：推力变化 / 推力换向的阈值判定。
- `bidirectional_motor_test/g10_udp.py`：G10 986-byte UDP 帧解析、10 kHz 样本时间重建和接收线程。
- `bidirectional_motor_test/motor_test_node.py`：ROS 话题、接收回调、定时发布和 CSV。
- `tests/test_control.py`：纯 Python 单元测试，无需连接电机。

```bash
cd ~/bidirectional/Bidirectional-Motor-Test
python3 -m unittest discover -s tests -v
```

单元测试不等于实机验证，尤其不验证 ESC 参数、RPM 数据方向、G10 采集链路或物理换向安全。
