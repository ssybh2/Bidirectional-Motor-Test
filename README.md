# Bidirectional Motor Test · 正弦换向与推力延迟记录

运行环境：**Ubuntu 22.04 / ROS 2 Humble**。本包是 [AIMEtherCAT/EcatV2_Master](https://github.com/AIMEtherCAT/EcatV2_Master) 的上层 ROS 控制节点，不负责产生物理 DSHOT600 波形。

## Ubuntu 原生图形界面（不用在终端里盯 CSV）

`g10-linux-udp` 分支增加了 **G10 推力测量** 桌面应用。它使用 Ubuntu 自带的 Tk 图形库，**不依赖浏览器、matplotlib 或 Windows 软件**。可以在 Ubuntu 的“应用程序”中点击启动，主界面包含实时推力大数字、30 秒滚动曲线、DSHOT 指令、G10 接收状态，以及 **8 路尚未标定的 ADC 原始数值**；ADC6 仅标为当前推力候选通道。其他 7 路不能擅自解释为扭矩、电压、转速等物理量。

它本身**不争用 UDP 4800，也不发布任何电机控制消息**：GUI 只读取已有 ROS 节点写出的 CSV。新增的 `g10_channels.csv` 每约 16 ms 保存一次八通道快照（默认每 4 个 G10 UDP 包取一个快照，约 62 Hz）；推力检测仍使用内部全部约 10 kHz 采样，`force.csv` 仍约 250 Hz。这些不同数据频率不能混为一谈。窗口上的 `去皮`、`砝码标定` 和 `查询标定` 按钮调用现有 ROS 服务，保留其 **DISARM、无动力和稳定窗口** 验证。

首次需要**在 Ubuntu 上执行一次**如下命令安装图形依赖、构建 ROS 包、注册桌面图标：

```bash
sudo apt update
sudo apt install -y python3-tk

cd /home/hby/bidirectional/Bidirectional-Motor-Test
git switch g10-linux-udp
git pull --ff-only origin g10-linux-udp

cd /home/hby/bidirectional
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to bidirectional_motor_test

cd /home/hby/bidirectional/Bidirectional-Motor-Test
bash scripts/install_g10_desktop.sh
```

之后从 Ubuntu 应用列表搜索 **G10 推力测量**，点击即可打开窗口；如果桌面也生成图标，首次可能需要右键 **允许启动（Allow Launching）**。如果不想安装快捷方式，也可以在已 source 的 ROS 终端执行一次 `ros2 run bidirectional_motor_test g10_dashboard` 打开窗口，以后所有数据和操作都在 GUI 内。

GUI **打开窗口本身不再依赖加载 ROS 2 环境**。桌面启动脚本先打开 Tk；只有点击启动采集、去皮或标定，才通过 `scripts/ros_env_exec.sh` 加载 ROS 环境并执行相应命令。如果 Ubuntu 图标点击无窗口，启动器会记录 Python 堆栈，并通过系统对话框/通知提示失败原因。查询最近一次启动日志：

```bash
tail -n 90 "${XDG_STATE_HOME:-$HOME/.local/state}/bidirectional-g10/dashboard-launch.log"
```

如需直接看启动报错，也可用 `bash scripts/launch_g10_desktop.sh`。用 `bash scripts/install_g10_desktop.sh` 更新桌面快捷方式；安装脚本会给 ROS 环境帮助脚本添加执行权限。

GUI 可以监测**已经运行的** `motor_test.launch.py`，或者点击 **启动采集** 来启动自己的 ROS 节点；它会先检查 G10 本地 IP 和 UDP 端口，避免与 `g10_probe`/其他节点争抢。点击 **停止采集** 只会关闭 GUI **自己启动的**节点，关闭窗口时也先请求该节点正常停机。对于在其他终端启动的节点，GUI 仅附着监看，不擅自停止。不要把软件停止按钮当成硬件急停。

- **去皮**：先卸载、静止两秒，确保遥控器开关 2、DSHOT 0、ESC 动力断开，点击 `空载去皮`；小于几个 ADC 计数的量化波动是正常的。
- **砝码标定**：先去皮，输入真实已知质量（kg），稳定加载后点击 `砝码标定`，成功才可切换显示为 kgf。手按未知力不能标定。
- **查看标定**：显示零点、kgf/count 及当前静态窗口状态。单位变更时曲线自动分段，不把计数和 kgf 混绘。
- **故障排查**：如果没有窗口，请确认 Ubuntu 图形桌面、`python3-tk`、ROS 已构建。桌面启动脚本为 `scripts/launch_g10_desktop.sh`；由 GUI 启动的 ROS 日志保存在 `~/bidirectional/measurements/g10_dashboard_ros.log`。

**网络前置条件不变**：G10 专用网卡 `enp5s0` 需有 `192.168.127.55/24`，EtherCAT 使用另一块物理网卡。GUI 不会用 sudo 配网，也不会自动启动/停止 EtherCAT 主站。UDP 接收样本时间仍是基于主机收到一批数据的估计值，不是硬件同步时间戳。

## 实际连接的话题

| 数据 | 话题 | ROS 消息 |
|---|---|---|
| DJI RC 输入 | `/ecat/sn2555957/app1/read` | `custom_msgs/msg/ReadDJIRC` |
| DSHOT 输出 | `/ecat/sn2555957/app2/write` | `custom_msgs/msg/WriteDSHOT` |
| 推力数据 | **本分支默认启用 G10 UDP 直接采集**；可选关闭并改为 `force_topic` | ADC 原始计数 / 可选 `std_msgs/msg/Float64` |
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

节点每个控制周期记录（并由独立的、有限批次的 G10 接收定时器处理 UDP）：

- `command.csv`：ROS 调用 publish 时的单调时钟时间、实际 DSHOT 指令、带符号的正弦量和模式。
- `force.csv`：推力 ROS 消息**抵达本节点**的单调时钟时间、数值和当时最新的 DSHOT。
- `event.csv`：解锁、正弦启动、换向等待以及**相反方向第一条非零命令**的时刻；每个测试事件分配 `event_id`。
- `g10_quality.csv`：每秒记录有效/无效包、队列丢包、估计时间倒退、包序号异常、最新收包年龄和自动归零进度。
- `test_*_raw_event_XXXX.csv`：每次首条正向/反向非零命令的前后完整原始采样窗口，后台写盘；不包含虚构的丢失采样。
- `latency.csv`：每个 `event_id` 的两个指标：`force_onset`（推力向目标方向变化超过阈值），`target_sign`（推力达到目标符号及阈值）。需要连续 `force_confirm_samples` 个样本满足条件；记录的是第一个满足条件的样本时间。

事件的 **t₀** 是 `first_command` / `reversal_command` 的首次非零 DSHOT 发布时刻，**不是**正弦过零、也不是开始强制等待的时刻。由此不会把人工设定的 `reversal_pause_sec` 直接算进“指令→推力”延迟。

```text
t0 = 首次反向非零 DSHOT publish（ROS 主机单调时钟）
t1 = 满足显著变化判据的第一条推力样本时刻
推力变化观测延迟 = t1 - t0

t2 = 连续达到目标符号阈值的第一条推力样本时刻
推力换向观测延迟 = t2 - t0
```

直接 G10 的显著变化与目标符号阈值默认分别为 50 counts，连续确认 10 个原生样本（1 ms）；`g10_raw_change_threshold` 和 `g10_raw_sign_threshold` 在标定后**仍按原始 counts 配置**，代码自动换算为 kgf。ROS 推力话题模式下，阈值单位与话题一致。否则噪声或原方向的惯性衰减可能造成“变化开始”的误判。

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

默认配置已经启用直接接收，并增加了数据健康联锁：

```yaml
force_topic: ""
g10_udp_enabled: true
g10_adc_channel: 6
g10_auto_zero: true
g10_auto_zero_samples: 10000
g10_force_sign: -1
g10_kgf_per_count: 0.0
g10_require_healthy: true
g10_no_packet_timeout_sec: 0.15
g10_capture_raw_events: true
g10_raw_pre_sec: 0.25
g10_raw_post_sec: 0.75
```

`g10_kgf_per_count: 0.0` 表示先用 `raw_count` 测量。启动前让台架完全卸载，节点用最初 1 秒的 10000 点自动归零。此模式的建议初始阈值为 50 counts、连续 10 点（1 ms）。接着放置已知质量 `M_kg` 的砝码，记录稳定后的 `raw_loaded` 和零点 `raw_zero`：

```text
g10_kgf_per_count = M_kg / abs(raw_loaded - raw_zero)
```

填入比例后，`force_unit` 自动变成 kgf；G10 专用触发阈值继续按 ADC counts 配置，由程序自动换算。`g10_force_sign` 决定哪一侧为正；如果正转推力显示为负，改成 `1`。

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
git fetch origin
git switch g10-linux-udp
git pull --ff-only origin g10-linux-udp

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

## 推 / 拉双向推力：修复有符号 ADC 边界回绕

此前用户观察到“推、拉都在纵坐标正半轴，一侧约几十/上千 counts，另一侧突然出现约 6.5 万 counts”。**现有抓包解码的是 signed int16；ADC6 空载约 32688，距最大正值 32767 仅 79 counts。** 因此，零点另一侧的原始数据很可能跨越 +32767/-32768 这个数字表示边界。旧算法直接 \`raw - zero\`，会把普通小变化算成数万 counts，甚至把反向力画到正半轴。

新版本默认开启 \`g10_adc_signed16_modulo: true\`，对原始 signed-int16 读数使用相对零点的最短模 65536 差值：

\`\`\`text
relative_counts = ((raw_adc - zero_raw + 32768) % 65536) - 32768
signed_force = g10_force_sign * relative_counts * gain
\`\`\`

结合你当前的 \`zero_raw ≈ 32688\`、\`g10_force_sign=-1\`，以下是**纯软件演示，不是实测标定结果**：

| 原始 ADC6 | 最短相对变化 | 软件显示 | 说明 |
|---:|---:|---:|---|
| 32688 | 0 | 0 raw_count | 零点 |
| 31088 | -1600 | +1600 raw_count | 一个力方向 |
| -32648 | +200 | -200 raw_count | 另一个方向，跨越 int16 边界 |

**两个方向能出现在零线两侧，不代表手按和手拉的真实大小相等。** GUI 现在强制以 0 为纵坐标中央、正负使用对称的自动量程，但绝不自动把两侧幅值“调到一样”。若想比较推/拉真实力，必须用已知的双向静态载荷核实通道编码、线性度和符号；单边砝码标定不足以认证双向测力。

**这个 modulo 处理依然是一项协议假设。** 它要求原始通道确实按 16 位循环数字编码、相对零点的真实变化在正负 32768 counts 之内。若 ADC 已经饱和而不是回绕、传感器本身不能承受反向力，或者真实范围超出半个周期，算法无法恢复物理力。原始 \`raw_force\` 和 8 路 ADC CSV 仍按线上的 signed-int16 数值原样保存，可对照厂家 Windows 软件。仅在无桨、ESC 动力切断的条件下先做小幅推拉验证：**原始 ADC 在 \`32767 → -32768\` 附近连续越界时，新推力曲线应平滑穿越 0，且符号相反**；若读数长时间卡在极值、异常跳跃或不随反向力变化，应停止使用该方向数据，继续抓包核查。

修改此配置后要**退出旧采集节点，启动新的测量会话**；旧 \`force.csv\` 的巨大尖峰不属于可直接重用的准确数据。旧版增益文件（schema v1）不会自动当成新版增益使用；遇到警告时请重新空载去皮、用已知质量按正确方向标定，新文件会记下 \`adc_delta_mode\` 以防混用。去皮的稳定窗口和自动零点也已经使用同样的循环差值，不会因刚好跨越边界而取出虚假的中间值。

## G10 空载去皮与已知质量标定（Ubuntu ROS 服务）

无需 Windows 软件。三种 ROS 2 服务均只处理 G10 数据，**不会主动启动电机**：

| 服务 | 作用 |
|---|---|
| `/bidirectional_motor_test/g10_tare` | 使用最近 1 秒约 1000 个抽取后的稳定 ADC 样本更新零点 |
| `/bidirectional_motor_test/g10_calibrate` | 使用已知质量（kg）计算并保存 kgf/count 增益 |
| `/bidirectional_motor_test/g10_calibration_status` | 显示零点、增益、单位、静态窗口状态 |

执行条件：**ESC 动力断开、无桨、遥控器开关 2（DISARM，或没有接入 RC）、DSHOT 0，且 G10 数据在线。** 切忌在带推力/转动中去皮。每次标定都要求连续新鲜的原始样本；若采样窗口峰峰值大于 `g10_stability_range_counts`（默认 10 ADC counts），服务会拒绝。

1. 启动节点，等待 `G10 auto-zero complete`，空载放置至少 2 秒；随后**去皮**：

```bash
source /opt/ros/humble/setup.bash
source ~/bidirectional/install/setup.bash
ros2 service call /bidirectional_motor_test/g10_tare std_srvs/srv/Trigger "{}"
ros2 service call /bidirectional_motor_test/g10_calibration_status std_srvs/srv/Trigger "{}"
```

   成功时会返回新的 `zero_raw` 和当前窗口的 ADC 峰峰值。去皮后空载信号应回到零附近，但小于一个 ADC 计数的量化波动和缓慢漂移仍然正常。此操作**保留既有增益，不会重新计算 kgf/count**。

2. 如果需要把原始计数变成 kgf，沿待测正向放置**实际已知质量** `M_kg` 的稳定载荷，例如 0.5 kg；确保重量的力作用方向与推力传感器的测量方向一致，不是手按。在放稳至少 2 秒后：

```bash
ros2 param set /bidirectional_motor_test g10_calibration_mass_kg 0.5
ros2 service call /bidirectional_motor_test/g10_calibrate std_srvs/srv/Trigger "{}"
ros2 service call /bidirectional_motor_test/g10_calibration_status std_srvs/srv/Trigger "{}"
```

   公式：`kgf_per_count = M_kg / (g10_force_sign * (ADC_loaded - ADC_zero))`。若增益为负向、已知载荷引起的变化太小（默认需至少 100 counts）、数据不稳定或正在转动，服务会拒绝。需要验证负载为真实重力产生的轴向力，不能只按砝码质量猜测机械施力大小。

3. 标定成功后，可卸载并复查 `force.csv`，其中 `force_unit` 将从 `raw_count` 改为 `kgf`。当前会话已有的原始行仍保留旧单位，**不要把两种单位混为一列连续曲线**。标定的时间与新旧系数记入 `event.csv`。

标定增益自动保存在 `~/bidirectional/calibration/g10_channel6.json`，下次启动时（`g10_load_calibration: true`）会验证设备 IP、ADC 通道和力方向后载入。**零点永远不从旧文件恢复**，每次启动都会重新自动归零，必要时再执行一次 `g10_tare`。若使用新设备或改变推力通道，必须重新确认增益；手动设置 `g10_kgf_per_count > 0` 可覆盖缓存的增益。

直接 G10 的触发阈值现在统一由 `g10_raw_change_threshold`、`g10_raw_sign_threshold` 指定，**始终以 ADC count 配置**。程序在切换到 kgf 时自动转换阈值，不会突然因单位变化失去检测。此前的 `force_change_threshold` 和 `force_sign_threshold` 留给 ROS `force_topic` 输入模式。

**反向测力重要限制：**ADC6 的空载值约 32688，接近 signed-int16 正边界。程序现在可以按“模 65536 回绕”的假设计算双向相对变化，但尚未凭真实双向静载确认它一定是回绕而非饱和，也未确认传感器的反向力范围。**一次正向砝码标定不等于双向推力已经可信。** 有桨反转前必须先以已知双向静载验证原始编码、测量量程与机械装夹。

## Ubuntu G10 UDP 首次部署：先验证收包，不要直接通电转桨

**必须使用两块独立物理网卡：EtherCAT 保持原专用网口，G10 接 Ubuntu 的另一块网卡。** 将命令中的 `<G10网卡名>` 替换为 `ip -br link` 查到的实际名称，务必不要改到 EtherCAT 网卡：

```bash
ip -br link
sudo ip address add 192.168.127.55/24 dev <G10网卡名>
sudo ip link set <G10网卡名> up

# 不需要 ROS，无桨、ESC 动力断开时就可完成。
cd ~/bidirectional/Bidirectional-Motor-Test
python3 -m bidirectional_motor_test.g10_probe --bind 192.168.127.55 --seconds 15
```

若确实收到经过校验的 G10 UDP，探针打印解码包数、包序号异常及每路 ADC 的 min/max。**探针不会发送任何启动或握手命令。** 如果打印 `NO DATA`，这不证明协议解析一定有误；也可能是厂家 Windows 软件启动时发送了初始化/订阅控制包。需要从 Windows 软件打开、连接到首包输出的完整双向 `.pcapng` 中分析，不能凭空拼装初始化指令。

探针与 ROS 节点**不能同时**绑定 UDP 4800；探针退出后才启动：

```bash
cd ~/bidirectional
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to bidirectional_motor_test
source install/setup.bash
ros2 launch bidirectional_motor_test motor_test.launch.py
```

确认日志出现 `G10 auto-zero complete`，并且台架空载/轻微施加已知静载时 `force.csv` 的方向、数据量有意义，之后再考虑命令测试。没有数据、自动归零未完成、收包超时、队列积压或包序号不连续时，默认 `g10_require_healthy: true` 会输出 0 并要求重新 **2 → 3 → 1**。若**仅在完全断开 ESC 动力的 ROS 联调**中不希望等待 G10，可暂时设置 `g10_require_healthy: false`；实测务必改回 `true`。

新增的采样窗口默认为首条反向非零 DSHOT 前 0.25 秒、后 0.75 秒，保存全部解码的约 10 kHz 样本（即使连续 `force.csv` 为每 40 点取 1 点）。`g10_quality.csv` 记录实际队列与丢包情况；任何显著时钟重叠、未标定 UDP 接收时延、设备 ADC 滤波延迟，都不能仅凭 0.1 ms 名义采样间隔消除。

**注意：接到同一主机只解决 Windows/Ubuntu 的独立时钟基准问题。** `g10_udp.py` 假设最新样本接近 UDP `recvfrom` 返回时刻，把前 39 个样本以 100 µs 间隔往回推。这是估算，不是设备硬件时间戳。厂家缓存、网络和 Linux 任务调度都能把估计的推力变化时刻移位。数据健康校验和完整采样窗口提高可信度，仍然需要硬件同步验证毫秒/亚毫秒级响应精度。

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
- `bidirectional_motor_test/g10_udp.py`：G10 986-byte UDP 帧解析、样本时间估计和接收线程。
- `bidirectional_motor_test/g10_health.py`：数据就绪 / 推力基准时间判断。
- `bidirectional_motor_test/raw_capture.py`：换向事件原生采样缓存与异步 CSV 写盘。
- `bidirectional_motor_test/g10_probe.py`：纯接收 UDP 诊断工具（无发包或初始化）。
- `tests/test_motor_node_callbacks.py`：不依赖 ROS 的真实控制回调 mock 回归测试。
- `bidirectional_motor_test/motor_test_node.py`：ROS 话题、接收回调、定时发布和 CSV。
- `tests/test_control.py`：纯 Python 单元测试，无需连接电机。

```bash
cd ~/bidirectional/Bidirectional-Motor-Test
python3 -m unittest discover -s tests -v
```

单元测试不等于实机验证，尤其不验证 ESC 参数、RPM 数据方向、G10 采集链路或物理换向安全。
