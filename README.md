# G10 Linux 推力上位机

**Ubuntu 22.04 · ROS 2 Humble · G10 UDP + EtherCAT DSHOT**

![G10 Linux 上位机：实时推力、8 路 ADC、DSHOT 与实验录制](assets/g10-dashboard.webp)

不用 Windows 上位机，即可在 Ubuntu 桌面查看 G10 推力、正负推拉曲线、8 路 ADC 原始值、DSHOT 指令及通信状态；提供**空载去皮、已知砝码标定、实验录制、指令→推力响应延迟 ZIP 导出**。

> **适用范围**：本分支按已抓取的 G10 UDP 报文开发。ADC 6 暂按推力通道处理；其他 ADC 的物理含义及反向量程尚待验证。未标定时读数单位为 `raw_count`，**不是 kgf**。

## 1. 连接 G10

G10 网线接 Ubuntu 的**独立物理网卡**；EtherCAT 主站使用另一块网卡，二者不可混用。以下使用实验中的 `enp5s0`，请按实际网口名替换：

```bash
ip -br link
sudo ip link set enp5s0 up
sudo ip address replace 192.168.127.55/24 dev enp5s0
ip -br -4 addr show enp5s0
```

已验证的 G10 数据流为 `192.168.127.56:5000 → 192.168.127.55:4800/UDP`。配置不应覆盖 EtherCAT 网口，也无需更改系统默认网关。以上 IP 配置为临时配置，重启后可能需要重新设置。
```bash
sudo ip link set enp5s0 up
sudo ip addr replace 192.168.127.55/24 dev enp5s0
```
## 2. 安装 ROS 包与桌面软件

前置条件：Ubuntu 22.04 已安装 **ROS 2 Humble**；可正常编译 `colcon` 工作空间；已具备项目使用的 `custom_msgs`（来自 [EcatV2_Master](https://github.com/AIMEtherCAT/EcatV2_Master)，可在同一个工作空间编译，或 source 已安装它的工作空间）。仅有本仓库、缺少 `custom_msgs` 时，ROS 节点无法编译。

```bash
sudo apt update
sudo apt install -y git python3-tk python3-colcon-common-extensions

# 新主机：将仓库放在工作空间根目录下，保持桌面脚本的路径约定
mkdir -p ~/bidirectional
git clone --branch g10-linux-udp \
  https://github.com/ssybh2/Bidirectional-Motor-Test.git \
  ~/bidirectional/Bidirectional-Motor-Test

# 如 custom_msgs 在另一个已编译的 ROS 工作空间，先 source 其 install/setup.bash
cd ~/bidirectional
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to bidirectional_motor_test

# 以普通用户注册 Ubuntu 桌面应用（不要用 sudo）
bash ~/bidirectional/Bidirectional-Motor-Test/scripts/install_g10_desktop.sh
```

已有本仓库时，无须重复 clone：在 `Bidirectional-Motor-Test` 下执行 `git switch g10-linux-udp && git pull --ff-only origin g10-linux-udp`，然后重新 `colcon build` 即可。

**桌面打开方式**：Ubuntu 应用列表搜索 **G10 推力测量**；也可运行 `bash ~/bidirectional/Bidirectional-Motor-Test/scripts/launch_g10_desktop.sh`。GUI 打开本身不需要先运行 ROS，但点击「启动采集」需要 ROS 环境及可用的 G10 UDP 流。

## 3. 启动与录制

1. **保持 ESC 动力断开、测试台空载**。Motor Test 不再自动占用 UDP 4800；打开 GUI →「启动采集」才启动独立 G10 进程。
2. 空载静止后点击「空载去皮」；使用真实已知砝码时才能执行「砝码标定」（单位才会切换为 `kgf`）。可先轻推/轻拉验证正负曲线。
3. 在「实验录制」中**选择保存目录 → 开始录制 → 结束并保存**。默认写入 `~/bidirectional/recordings/` 下的 ZIP；内含 `command.csv`、`force.csv`、`timeline.csv`、`event_summary.csv`、`latency.csv` 和采集质量/说明文件。原始连续日志保存在 `~/bidirectional/measurements/`。
4. 如果需要实际 DSHOT 指令测试，**另行启动并验证 EtherCAT 主站与 Motor Test**；本 GUI 只启动、停止 G10 采集，绝不会启动或停止电机控制。配置中默认遥控器话题 `/ecat/sn2555957/app1/read`、DSHOT 话题 `/ecat/sn2555957/app2/write`，设备 SN 不同须先修改 [config/motor_test.yaml](config/motor_test.yaml) 并重新编译。原控制安全顺序为遥控器 **2 → 3 → 1**；带桨反转前仍需真实 RPM 停转联锁与独立断电急停。

**延迟含义**：报告的是 ROS DSHOT **发布时刻 → G10 估算的推力采样时刻**，不是硬件同步的纯电机延迟；G10 内部缓存、网络和调度误差仍需单独标定。没有有效推力响应时结果标记为未确定，不会写成 `0 ms`。

## 4. 检查与排错

```bash
# 没有 G10 数据：确认网卡、IP 和 UDP 报文
ip -br -4 addr show enp5s0
sudo tcpdump -ni enp5s0 'udp port 4800'

# 图标点击没反应：查看桌面启动日志
tail -n 80 "${XDG_STATE_HOME:-$HOME/.local/state}/bidirectional-g10/dashboard-launch.log"

# 只读 UDP 探针（必须先停止正在接收 UDP 的 ROS 节点）
cd ~/bidirectional/Bidirectional-Motor-Test
python3 -m bidirectional_motor_test.g10_probe \
  --bind 192.168.127.55 --seconds 10
```

如提示 `Permission denied` 无法编译，请检查以前是否用 root 生成了 `build/`、`install/`、`log/`；**不要用 sudo 执行 colcon 或启动 GUI**。

更多协议说明、静态标定、换向状态机与日志字段见 [详细技术文档](docs/DETAILED_GUIDE.md)。

## 5. root EtherCAT + 普通用户 G10 GUI（UDP 4800 冲突修复）

**一个 G10 测试台只能有一个 UDP 4800 采集者。** 本 GUI 只读取 CSV，绝不另外抢占 UDP。现在 `ros2 launch bidirectional_motor_test motor_test.launch.py` 会从 **bidirectional_motor_test 的 colcon 安装前缀**计算工作空间，因此 root 与普通用户运行时均使用：

- `<工作空间>/measurements/`（CSV 与实时 GUI）
- `<工作空间>/calibration/g10_channel6.json`（标定增益）

此规则会覆盖 YAML 中的 `~/...` 直接运行回退值；不依赖 root 的 `HOME`。如果工作空间为 `/home/hby/bidirectional`，启动日志应出现 `CSV=/home/hby/bidirectional/measurements/...`，而不是 `/root/bidirectional/measurements/...`。旧 root 目录中的历史记录不会自动迁移。

**推荐启动顺序（先断开 ESC 动力、拆桨检查）：**

```bash
# 终端 1：需要原始网卡权限的 EtherCAT 主站按既有方式启动
sudo -s
source /opt/ros/humble/setup.bash
source /home/hby/one/install/setup.bash
ros2 launch soem_bringup bringup.launch.py

# 终端 2：普通 hby 用户，先确认能收到 RC/DSHOT 话题
source /opt/ros/humble/setup.bash
source /home/hby/one/install/setup.bash
source /home/hby/bidirectional/install/setup.bash
ros2 topic echo /ecat/sn2555957/app1/read --once
ros2 launch bidirectional_motor_test motor_test.launch.py

# 终端 3：普通 hby 用户打开 GUI，不要再启第二个采集节点
bash /home/hby/bidirectional/Bidirectional-Motor-Test/scripts/launch_g10_desktop.sh
```

如果 Motor Test 确实必须以 root 运行，先确保 `/home/hby/bidirectional/measurements` 及 CSV **对 hby 可读**，标定目录也有合适的共享写权限。历史上由 root 建立的目录可能无法让普通用户创建新会话，务必停机后检查所有权；不要以 root 启动图形界面。正常情况下 DDS 消息订阅/发布本身不需要 root，两端仍须使用同一 ROS_DOMAIN_ID 与兼容的 RMW 环境。

GUI 在点击「启动采集」时，只检查 **g10_*_g10_quality.csv** 采集心跳；Motor Test 的控制心跳不再阻止独立采集。若 UDP 已占用且找不到采集心跳，会提示检查 `sudo ss -lunp | grep ':4800'`，不会尝试抢占端口。采集进程必须唯一：不要同时运行 g10_probe 或另一套绑定 4800 的厂商软件。

`ros2 run` **绕过了上述 launch 参数覆盖**；若直接运行，请显式指定绝对路径 `log_directory` 和 `g10_calibration_file`，避免再次遇到 root/普通用户 HOME 不一致。

Ctrl+C 后 ROS 2 可能先销毁 DDS 上下文，使最后的 DSHOT 0 **无法发出**。现在退出时会检测上下文、捕获发布异常并继续释放 UDP 与 CSV，但这只解决退出异常，**不构成物理急停，也不保证电机已停转**。请使用独立物理 ESC 动力切断及下位机失联清零措施。


## 6. GUI 手动采集与电机控制独立运行

新版 `motor_test.launch.py` 只负责 `/bidirectional_motor_test` 遥控器与 DSHOT 控制，它不绑定 UDP 4800，也不自动去皮或自动开启 G10 采集。它额外发布 `/bidirectional_motor_test/command_meta`，提供同一主机单调时钟下的 DSHOT 发布时间、正弦状态和首次换向事件。它仍遵守原来的 RC 2 → 3 → 1 解锁顺序以及可选 RPM 停转联锁。

**只有 GUI「启动采集」才启动 `g10_capture.launch.py` 中的 `/g10_acquisition`。** 采集进程负责 UDP 4800、G10 ADC/自动归零、CSV、标定服务、指令与推力关联。GUI「停止采集」只退出该进程，Motor Test 仍然继续控制电机。GUI 只显示 `g10_...` 测量会话，控制日志另存为 `control_...`，不再将控制日志当成采集心跳。

```bash
# 更新代码后，普通用户编译：
cd /home/hby/bidirectional/Bidirectional-Motor-Test
git switch g10-linux-udp
git pull --ff-only origin g10-linux-udp
cd /home/hby/bidirectional
source /opt/ros/humble/setup.bash
source /home/hby/one/install/setup.bash
colcon build --symlink-install --packages-select bidirectional_motor_test

# 已有正常 EtherCAT 主站的前提下，在独立控制终端（可保持现有 root 方式）：
source /opt/ros/humble/setup.bash
source /home/hby/one/install/setup.bash
source /home/hby/bidirectional/install/setup.bash
ros2 launch bidirectional_motor_test motor_test.launch.py

# Ubuntu 普通用户打开 GUI，窗口出现时尚未启动采集：
bash /home/hby/bidirectional/Bidirectional-Motor-Test/scripts/launch_g10_desktop.sh
# 点击「启动采集」后才会有 UDP 4800 接收。
```

单独检查采集进程也可以在另一普通用户终端运行 `ros2 launch bidirectional_motor_test g10_capture.launch.py`，但 **GUI 只能停止由 GUI 自己启动的采集进程**。标定服务改为 `/g10_acquisition/g10_tare`、`/g10_acquisition/g10_calibrate`；必须收到来自 Motor Test 的新鲜 `DISARM` 和 DSHOT 0 心跳，仍须切断 ESC 动力并使用实际已知载荷。采集和控制必须运行在**同一台 Ubuntu 主机**上，因为 Linux `monotonic_ns` 跨主机不能直接比较；各 ROS 进程也须使用兼容的 `ROS_DOMAIN_ID` / RMW 环境。

采集服务用指令元数据中的原始发布时刻，与缓存的 G10 原生样本做时间关联。元数据缺失、推力基线不新鲜时**不伪造测量结果**，现有 CSV/ZIP 导出格式保持不变。

**安全变化：** 现在 G10 数据中断或 GUI 停止采集 **不会令 Motor Test 停机**。GUI 的「停止采集」不是急停，正式带桨测试必须具备独立物理急停、RPM 停转联锁和下位机失联清零措施。Ctrl+C 时若 ROS 上下文已失效，软件也无法保证最后的 DSHOT 0 已发布。

## 7. GUI 的 DSHOT 显示与 G10 指令元数据诊断

控制节点和采集节点拆分后，GUI「当前 DSHOT」直接读取 **Motor Test 产生的** `control_*_command.csv` 最新有效记录，而不是 G10 `force.csv` 中可能滞后的 `last_dshot`。因此，即使 GUI 尚未点击「启动采集」，依然可以读取已经运行的控制日志并显示 DSHOT。控制日志缺失、不可读或超过两秒未更新时显示 **—（未知）**，不能用 0 假装停机。这里显示的是 ROS **已发布的命令**，不是从 ESC 读取的真实输出或转速。

G10 接收器仍然通过 ROS `/bidirectional_motor_test/command_meta` 获取精确的发送时间戳，供指令与推力延迟分析使用。GUI 采集状态会单独显示「指令元数据在线」或「指令元数据未同步」。**G10 UDP 正常不代表 ROS 指令元数据已经同步**：若后者异常，实时推力仍可显示，但不应把该会话用于可信的 DSHOT→推力延迟结论。

排查时，在同一 Ubuntu 主机检查：

```bash
source /opt/ros/humble/setup.bash
source /home/hby/one/install/setup.bash
source /home/hby/bidirectional/install/setup.bash

# Motor Test 的命令总线：在开关 2/3/1 变化时看 dshot 与 mode
ros2 topic info /bidirectional_motor_test/command_meta -v
ros2 topic echo /bidirectional_motor_test/command_meta --once

# 两个进程使用同一个 ROS_DOMAIN_ID / 兼容的 RMW 实现
printenv ROS_DOMAIN_ID RMW_IMPLEMENTATION
# GUI 启动的采集使用 scripts/ros_env_exec.sh 加载 ROS 环境

# 分别检查控制方和采集方最后一行（按文件修改时间排序）
find /home/hby/bidirectional/measurements -maxdepth 1 \
  -name 'control_*_command.csv' -printf '%T@ %p\n' | sort -nr | head -1
find /home/hby/bidirectional/measurements -maxdepth 1 \
  -name 'g10_*_command.csv' -printf '%T@ %p\n' | sort -nr | head -1
```

如果控制 CSV 非零、采集 CSV 始终空白，说明实时显示问题和 ROS 元数据传输问题是两回事。优先检查 root 和普通用户进程的 ROS_DOMAIN_ID/RMW 是否一致、`ros2 topic info -v` 的 publisher/subscriber 计数，以及普通用户是否对 `control_*_command.csv` 具有读取权限。不要仅凭 `SINE` 模式判定某一瞬间的 DSHOT 必定非零：换向等待与正弦死区本来就会输出零。

## 8. DSHOT 时间戳、双进程同步和 ZIP 缺失字段修复

### 数据流和具体修复

Motor Test 的 `control_*_command.csv` 本来就记录了每条 DSHOT 的 **`mono_ns = time.monotonic_ns()`（紧靠 `publisher.publish` 之前）**、`wall_ns`、`mode`、`dshot`、`logical_direction`。它是 **ROS 指令发布时间**，不是 H750/ESC 的实际执行时间。此前 GUI 只打包采集进程的 `g10_*_command.csv`；跨 root/桌面进程的 ROS 元数据未送达时，就会得到「推力正常但 `command.csv`、`event.csv`、`latency.csv` 全空」的 ZIP。这不是 Motor Test 没有时间戳。

现在采用两层恢复：

1. **运行期间**：G10 收到 ROS `/bidirectional_motor_test/command_meta` 后仍采用原消息。每 200 ms 还会只读扫描同一工作空间内持续刷新的 `control_*_command.csv` 和 `control_*_event.csv`；通过真实 `mono_ns` 去重/验证，补偿丢失的消息。补偿不创建 DSHOT 发布者，不控制电机。迟到的换向事件根据历史命令时间戳验证，不再只比较「最新命令」导致丢弃。补偿的数据源写在 `g10_*_command.csv` 的 `metadata_source` 字段（`ros` / `control_csv`）。
2. **导出时**：按实际 `mono_ns` 覆盖的录制区间查找唯一 `control_*_command.csv`，优先用 Motor Test 原始命令构建 ZIP 的 `command.csv` 和 `timeline.csv`。若有原始换向事件而采集端没收到，则把**真实命令 t0** 写入 `event.csv` / `event_summary.csv`，但标记 `metadata_unavailable`，**绝不杜撰推力延迟**。多个控制会话重叠时直接拒绝猜测来源；同时检查控制端与 G10 录制端的 wall-minus-monotonic 时钟偏移，防止电脑重启后相同开机时长对应的旧控制日志被误匹配。

录制开始前 GUI 会提醒是否只有推力而没有 DSHOT；保存时对「指令存在但延迟未测到」给出醒目的警告。在 ZIP 的 `metadata.json` 检查 `command_source`、`controller_session`、`recovered_control_references`、`latency_status` 和 `data_integrity_warnings`。需要注意：`latency.csv` 只有在采集端实际识别到事件时才有结果；未发生换向、未能确认新鲜基线、尚未到检测阈值或实时元数据持续失败，都可能使其**合理地为空**，这与丢失 DSHOT 指令不同。

### 现场验证与故障排查

先使台架断开 ESC 动力，再分别启动 EtherCAT、Motor Test、GUI「启动采集」。**两者须在同一台 Ubuntu 主机**，使用同一单调时间基准，并共享 `/home/hby/bidirectional/measurements`。先查看发布者/订阅者：

```bash
source /opt/ros/humble/setup.bash
source /home/hby/one/install/setup.bash
source /home/hby/bidirectional/install/setup.bash
ros2 topic info /bidirectional_motor_test/command_meta -v
ros2 topic echo /bidirectional_motor_test/command_meta --once
```

控制节点和 G10 接收节点分别由 `control_...` 和 `g10_...` 命名。下面的命令可以快速查看它们的原始计数；替换为当前最新前缀即可：

```bash
cd /home/hby/bidirectional/measurements
ls -lt control_*_command.csv g10_*_command.csv | head -8
tail -n 3 "$(ls -t control_*_command.csv | head -1)"
tail -n 3 "$(ls -t g10_*_command.csv | head -1)"
tail -n 3 "$(ls -t g10_*_g10_quality.csv | head -1)"
```

`g10_quality.csv` 新增 `metadata_source`、`command_age_ms`、`ros_commands`、`csv_recovered_commands`。如果 ROS 订阅者发现不到发布者，仍要核对 root 与普通用户环境下的 `ROS_DOMAIN_ID` / `RMW_IMPLEMENTATION` / DDS 网络配置；CSV 只读补偿只是降低数据丢失，**不会修复 DDS 本身**。检查文件权限：普通用户必须能读取 root 创建的 `control_*` CSV。若监测不到真实控制指令，录制仍可用于砝码标定，但不能用于指令→推力响应分析。

**针对 2026-09-27 约 ±32 raw_count 的实验**，采集侧起效阈值从 50 counts 调整为 8 counts（仅测量阈值，不改变 DSHOT/正反转安全联锁）。这是实验性起点，应先在空载及固定工况评估误报，单凭一次改参不能断言真实反转延迟。G10 ADC 时间是根据 UDP 到达时间估算的，不是硬件同步时钟；`latency_ms` 是 ROS `publish` → G10 估算样本变化的**观测延迟**，包含传感器、UDP 和调度偏差。

### 重新导出已录的旧 ZIP（不会修改原 ZIP）

若源 G10 会话和 Motor Test 的控制 CSV 都还留在 `measurements`，可以不用重新做实验：

```bash
cd /home/hby/bidirectional/Bidirectional-Motor-Test
python3 scripts/reexport_g10_zip.py \
  /home/hby/bidirectional/recordings/g10_record_20260927_140117_892738_9734.zip \
  --measurements /home/hby/bidirectional/measurements \
  --output /home/hby/bidirectional/recordings
```

新 ZIP 会保留原始 `mono_ns`。如果之前因 ROS 元数据没到达而没有原生换向窗口或推力检测结果，重新导出只能补回真实控制指令和事件参考，不会反向创造 10 kHz 推力起效时间。若原始 `control_*` 文件被删、跨主机录制、或多个控制会话时间重叠，不能可靠地补回指令数据。

**G10 时间戳质量：** 新版 ZIP 会统计 `g10_quality.csv` 在录制区间内新增的 `timestamp_regressions`、UDP 队列丢包、序号断档，并放入 `metadata.json` 和导出警告。G10 逐包时间由 UDP 接收时刻估算，相邻包可能出现重叠；被过滤的样本不应虚构补齐，因此不能仅依据软件估算时间戳宣称亚毫秒级电机响应精度。


## 9. G10 估算采样时间轴及迟到事件窗口修复

2026-09-27 的测试记录中，利用 UDP 包序号和主机接收时间回归得到约
**100993.4 ns/样本（约 9902 Hz）**。此前采用的每包独立
100000 ns/样本倒推方法会在普通接收抖动时造成相邻估算时间倒退，
把实际收到的 ADC 样本误判为过期并丢弃。

新版 G10 采集采用按包序号推进的单调估算时间轴：
第一包由接收时间锚定，后续按真实包序号差与经实测修订的
100993 ns/样本推进，不再让每次 UDP 接收抖动重新定位整包。
真实包跳号仍保留时间空洞，不伪造缺失包；重复和乱序包拒绝处理。
G10 quality 新增 sample_period_ns、sequence_clock_residual_ms、
sequence_clock_max_abs_residual_ms，以便识别模型漂移和调度异常。

延迟计算依旧是 ROS 指令发布到**估算** G10 采样的时间差：
起始硬件/网络延迟未知，周期也只有实验性估计值，不能等同于
硬件同步的 ESC 接收时刻或纯粹的电机响应时间。

原生事件窗口的历史缓存现覆盖前窗、最多两秒元数据滞后和后窗。
当 G10 采集收到迟到的命令事件时，会从缓存补回已经到达的
**命令前后**全部实测样本；若后窗已完整到达，事件立即完成并保存。
原始 UDP 序号和 ADC 值保持原样。旧 ZIP 缺失的样本无法事后恢复。

这些变化仅影响 G10 时间估计、测量和记录，不改变 Motor Test
的 DSHOT 输出、正反向控制参数、急停或联锁。更新时先物理断开
ESC 动力并关闭旧 GUI / 采集节点，重新启动后再做静态验证。


## 10. 换向延迟：第一条反向 DSHOT → G10 确认反向推力

**这是与旧 force_onset、target_sign 不同的新测量指标。**

- 起点 t0：Motor Test 在同一轮 SINE 运行中首次发布与上一方向相反的
  非零 DSHOT 指令的 ROS 单调时钟时间。中间的 DSHOT 0、固定停转
  等待和首次启动均不计入换向起点。
- G10 将每个实际收到的 UDP 包的 40 个推力点取中位数。中位数进入
  新方向且超过 g10_reversal_raw_sign_threshold（默认 4 raw_count），
  并连续保持 g10_reversal_stable_sec（默认 0.20s、至少 10 包），
  才算检测到持续的推力方向变化，避免 1ms 瞬态尖峰被当成换向完成。
  指标中的 observed_mono_ns 是持续变化的第一个 UDP 包的接收时刻，
  confirmed_mono_ns 是稳定性确认结束时刻。虽然会等确认结束再输出，
  latency_ms 仍以第一次接收时刻为终点，不额外计入稳定等待时长。
- 首次正向启动不计入反转。没有前置有效推力、发出 t0 之前已是目标
  推力方向、数据流中断、控制模式退出以及没有达到稳定条件的事件
  均保留失败或未测量状态，绝不填充虚假的延迟数字。

新 ZIP 的 reversal_summary.csv 是直接查看正→反、反→正延迟的表，
包含指令发出时间、首次持续反向推力观测时间、确认时间、测量值
和状态。event_summary.csv、latency.csv 也保留对应信息。GUI 实时
延迟及最终汇总优先显示新的 force_direction_change，而不把旧
force_onset 作为换向延迟。

**时钟边界：** G10 的 sequence-paced ADC 时间轴曾出现「估算样本
时间晚于数据包接收时间」的不可能现象。为避免虚假亚毫秒精度，
新指标的终点使用主机实际收包的 time.monotonic_ns()，因此反映
的是「ROS 发布时间 → Linux 主机观测到持续目标推力」。
它包含设备采样、打包、UDP 网络和系统调度延迟，不是 ESC 实际
执行或转子 RPM 完成反转的硬件同步测量。如果需要纯物理电机延迟，
必须额外校准测量链或增加共同硬件触发。

这个变更**只修改 G10 测量与 ZIP 输出**，不修改 Motor Test 的
正弦、换向暂停、RPM 保护或任何 DSHOT 安全行为。升级时先断开
ESC 动力并完全退出旧 GUI/采集节点；若本地 motor_test.yaml 有
未提交的实验参数，备份并 stash，再 git pull，最后合并检查参数。


### 原生 ADC 验证窗口

旧的原生事件窗口只在 t0 之后记录 0.75 秒，而确认推力反转
有时需要 1～2 秒。新版只对真正的正反方向切换，把该事件的
raw_events 原生 ADC 后窗口延长至 force_latency_timeout_sec
（默认 5 秒）；首次启动仍保留原来的短窗口。
因此可以从真实 ADC 包数据核对持续反向推力的检测过程，
无需推测缺失的后半段波形。该修改仅增加测量日志，注意
ZIP 体积和内存占用会相应增加。


## 11. 新增恒值交替 DSHOT STEP 模式（原正弦模式仍为默认）

Motor Test 共用原本的 WriteDSHOT 发布路径、50 Hz 定时器、RC
DISARM→ARM→RUN 授权、DSHOT 0 失控处理、事件时间戳及 G10
独立采集。修改 config/motor_test.yaml 的 waveform_mode 可在
sine（默认，原功能完全保留）和 step（恒值交替）之间切换。

STEP 的基础参数（与正弦模式互不干扰）：

~~~yaml
waveform_mode: "step"
step_forward_dshot: 1250
step_reverse_dshot: 250
step_hold_sec: 2.0
step_reversal_pause_sec: 2.0
step_allow_direct_reversal: false
~~~

上面为安全的停转等待配置：正向 2s → DSHOT 0 2s →
反向 2s → DSHOT 0 2s，以此循环。这里的 1250 和 250
分别相对于正反向 3D 区间起点 1048、48 偏移 202，
**只保证相等的 DSHOT 控制量，不保证物理推力相等**。
实际仍应由 G10 测试台标定和验证。

需要在固定的、具有独立物理急停的安全实验环境中研究
**正向非零命令直接切换成反向非零命令**，还需显式填写：

~~~yaml
step_reversal_pause_sec: 0.0
step_allow_direct_reversal: true
~~~

此时每 2s 在 1250 与 250 间切换，不发中间 DSHOT 0，
但实际时间会受 ROS timer 调度粒度影响。RPM 联锁若启用且
未通过，始终优先保持 0 直至停转条件通过。该直接换向
**不证明电机已停转或 ESC/桨叶能够承受突然反转**；
首次验证请断开 ESC 动力、拆桨，在 ROS 话题及 command.csv
中检查正反编码和对应 reversal_command 事件。
RC 2/3 停机、RC 断线或超时的零指令逻辑保持不变。

STEP 的发布消息及 event/command 行明确记作 mode=STEP，
G10 的 ROS 元数据和控制 CSV fallback 已同时支持此模式，
因此 reversal_summary.csv 可继续关联第一条反向非零 DSHOT
发布时间。其他参数和具体更新步骤见 docs/DETAILED_GUIDE.md。


## 12. RAMP 梯形斜坡换向（独立于原 STEP / SINE）

把 config/motor_test.yaml 的 waveform_mode 改为 "ramp"：
 
~~~yaml
waveform_mode: "ramp"
positive_peak_dshot: 1250
negative_peak_dshot: 250
ramp_rise_sec: 2.0
ramp_fall_sec: 2.0
ramp_positive_hold_sec: 2.0
ramp_negative_hold_sec: 2.0
ramp_zero_pause_sec: 0.0
publish_rate_hz: 50.0
~~~

此例：从 0 起步逐渐升到正向 1250（初次约 1s）→ 保持
2s → 线性指令下降，约 2s 后到反向 250（经过 DSHOT 0）→
保持 2s → 线性上升，约 2s 后回到正向 1250 → 循环。
上升和下降时间分别可调，例如 rise_sec=1、fall_sec=3；
数值表示完整的谷值到峰值、峰值到谷值的时长。
正反峰值使用现有 positive_peak_dshot/negative_peak_dshot；
原 step_forward_dshot/step_reverse_dshot 仅在 STEP 生效。
可将 ramp_negative_hold_sec: 0.0 取消谷值平台。

这里的“线性”仅指 3D DSHOT 区间内的**编码幅值**线性，
不保证电机转速或 G10 推力线性。由于 DSHOT 正、反两侧
编码区间分开，在每次过零时必定至少发送一轮 DSHOT 0；
因此编码数值本身不是一条跨越 0 的普通数轴直线。
调度粒度为 1/publish_rate_hz（50 Hz ≈20 ms）。
可通过 ramp_zero_pause_sec 设置额外的零指令等待。
若启用真实 require_rpm_for_reversal 联锁，过零等待优先
服从 RPM 已停条件；这会延长实际斜坡用时。
没有可靠 RPM 时，不得将斜坡过零或发送 0 当作转子静止。

DSHOT 日志标识为 RAMP，保留独立 G10 采集、
control CSV 后备元数据、首次反向非零 DSHOT 的
reversal_command 和 force_response_reference 时间戳。
请注意这一 t0 是**已进入新方向的指令**，而不是提前
开始减速斜坡的时刻；若比较完整动态响应，应同时从
command.csv 提取斜坡起点和零点。已有
reversal_summary.csv 报告的是指令侧新方向首次出现
到 G10 新方向推力被确认的时间，不是整个梯形波
从峰值降到谷值的历时。SINE/STEP 算法不变。
