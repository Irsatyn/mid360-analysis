# MID360 + FAST-LIO 数据分析

基于 Ubuntu 22.04、ROS2 Humble 的 C++17 工程。启动 MID360 驱动与指定 FAST-LIO，录制传感器/里程计/TF 数据，并通过两个自写节点分析 IMU、运动轨迹和 TF 一致性。

基础代码、自动化测试及 MID360 实时链路测试已完成，验证结果见 [验证记录](docs/validation.md)。已用真实数据验证驱动、FAST-LIO、分析节点和录包；定位精度与规定路线的运动验收仍需单独完成，见 [证据清单](docs/evidence/README.md)。合成测试数据不作为实机证据。

## 环境与第三方依赖

- Ubuntu 22.04、ROS2 Humble、C++17、`colcon`、`ament_cmake`。
- 实机链路需要 Livox-SDK2、PCL、Eigen、`livox_ros_driver2` 和 `fast_lio`。
- 分析包本身不链接 Livox/PCL，可在尚未安装第三方包时单独编译、运行测试和分析已记录的标准消息。
- 驱动使用提供的 `livox_ros_driver2.zip` 中 `livox_ros_driver2_humble/src`；FAST-LIO 使用提供的 `FAST_LIO.zip` 中 `FAST_LIO`。不要用其他版本替换后直接假定接口相同。
- 上游参考：[Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)、[livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)、[FAST-LIO ROS2](https://github.com/Ericsii/FAST_LIO)。实际运行以提供的 ZIP 为准。

系统包可在确认软件源后安装：

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-pytest \
  ros-humble-rclcpp ros-humble-sensor-msgs ros-humble-nav-msgs \
  ros-humble-geometry-msgs ros-humble-visualization-msgs \
  ros-humble-tf2-ros ros-humble-tf2-geometry-msgs ros-humble-tf2-tools \
  ros-humble-rviz2 ros-humble-ros2bag ros-humble-rosbag2-storage-default-plugins \
  ros-humble-ament-cmake-gtest ros-humble-ament-cmake-pytest \
  ros-humble-pcl-ros ros-humble-pcl-conversions libpcl-dev libeigen3-dev \
  python3-yaml python3-matplotlib python3-dev libapr1-dev
```

Livox-SDK2 的系统级安装按照上游 README 执行；驱动 CMake 要求能找到 `liblivox_lidar_sdk_shared.so`、`livox_lidar_api.h` 和 `livox_lidar_def.h`。安装到 `/usr/local` 后通常需要执行 `sudo ldconfig`。

## 编译分析包

以本仓库位于 `/home/simuel/mid360` 为例：

```bash
cd /home/simuel/mid360
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mid360_analysis
source install/setup.bash
colcon test --packages-select mid360_analysis --return-code-on-test-failure
colcon test-result --verbose
```

每个新终端都需要 source ROS 和工作空间。自动化集成测试会启动本机 DDS 通信，使用独立 ROS_DOMAIN_ID，不需要雷达。如果执行环境禁止网络接口访问，纯统计测试仍可运行，DDS 测试需在允许本机通信的终端执行。

## 准备完整实机工作空间

也可以将分析包和提供的两个第三方包放入独立工作空间：

```bash
mkdir -p ~/mid360_ws/src
ln -s /home/simuel/mid360/mid360_analysis ~/mid360_ws/src/mid360_analysis
unzip /home/simuel/mid360/FAST_LIO.zip -d ~/mid360_ws/src
unzip /home/simuel/mid360/livox_ros_driver2.zip -d ~/mid360_ws/src
mv ~/mid360_ws/src/livox_ros_driver2_humble/src ~/mid360_ws/src/livox_ros_driver2
source /opt/ros/humble/setup.bash
cd ~/mid360_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install --packages-up-to livox_ros_driver2 fast_lio mid360_analysis
source install/setup.bash
```

上述解压步骤只需做一次，目标目录已有内容时不要重复覆盖。提供的驱动 ZIP 已带 ROS2 Humble 的 CMake/package.xml，未发现 `build.sh`，因此直接使用 colcon；只有使用其他上游分发版本时才需要按其 README 执行 `build.sh humble`。第三方代码不纳入本仓库提交。

## 数据流与坐标系

```text
MID360 → livox_ros_driver2 → /livox/lidar (CustomMsg) ─┐
                          → /livox/imu (Imu) ────────┤→ fastlio_mapping
                                   │                └→ /Odometry /path /cloud_registered /tf
                                   │                            │
                                   └→ imu_stats_node            └→ odom_analyzer_node
                                          └──────── /analysis/* ────────┘ → RViz / rqt_plot

关键输入输出 → rosbag record → rosbag play --clock → 两个分析节点（use_sim_time=true）
```

```text
camera_init                 FAST-LIO 世界坐标系，由初始化确定，不是地理坐标
└── body                    IMU 机体系，FAST-LIO 发布动态 TF
    └── livox_frame          雷达坐标系，本项目从固定外参配置发布静态 TF
```

默认平移为 `[-0.011, -0.02329, 0.04412]` m，旋转为单位阵，方向是 LiDAR 到 IMU 的位姿，即 `body → livox_frame`。`config/mid360.yaml` 从提供的 FAST-LIO 配置复制，只把在线外参估计和 PCD 自动保存关闭，避免静态 TF 与在线外参不一致、长录制累积 PCD。第三方源码未修改。

静态 TF 从同一份 FAST-LIO YAML 读取平移和旋转，不在 Launch 中另写一套外参。开启在线外参估计时需要 `with_static_tf:=false`，否则启动会明确报错。自定义 frame 还应同步修改参数和静态 TF 启动方式。

## 主要输入话题

| 话题 | 类型 | 频率/含义 | frame / 发布者 |
|---|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | 配置及本次实测约 10 Hz | `livox_frame` / driver |
| `/livox/imu` | `sensor_msgs/msg/Imu` | 本次录包约 200 Hz | `livox_frame` / driver |
| `/Odometry` | `nav_msgs/msg/Odometry` | 本次实测约 10 Hz | `camera_init` → `body` / FAST-LIO |
| `/path` | `nav_msgs/msg/Path` | 按 FAST-LIO 实现更新 | `camera_init` / FAST-LIO |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | 配准点云 | `camera_init` / FAST-LIO |
| `/cloud_registered_body` | `sensor_msgs/msg/PointCloud2` | 机体系点云 | `body` / FAST-LIO |
| `/tf` | `tf2_msgs/msg/TFMessage` | 动态变换 | FAST-LIO |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 固定外参，transient_local | 本项目 static publisher |

提供的 FAST-LIO 源码注释掉了 `/Laser_map` 的实际发布调用，不能因为话题已注册就认为它有数据。RViz 中 Map 显示默认关闭，主要观察 `/cloud_registered`。

`bringup` 使用 `xfer_format=1` 明确选择 CustomMsg，以保留 FAST-LIO 所需的逐点时间信息。CustomMsg 不能直接作为 RViz PointCloud2 显示；这里通过 FAST-LIO 配准点云可视化。

## 两个自写节点

### IMU 统计节点

订阅 `/livox/imu`，使用固定窗口统计三轴加速度和角速度、频率、时间间隔抖动、疑似丢帧间隔、非递增时间戳；完整窗口满足加速度模长接近 1 g、角速度小和模长标准差小时判定为静止。静止时更新陀螺零偏，运动时保留最近一次估计。

| 输出 `/analysis/imu/…` | 类型 | 单位/含义 |
|---|---|---|
| `rate` | Float64 | Hz，`1/mean(正时间间隔)`，未有间隔时 0 |
| `dt_jitter` | Float64 | s，时间间隔总体标准差 |
| `is_static` | Bool | 完整窗口的静止判定 |
| `acc_norm` | Float64 | g，加速度模长窗口均值 |
| `gyro_bias` | Vector3Stamped | rad/s，最近静止估计及其时间戳；首次静止前为 NaN |
| `stats` | Float64MultiArray | 17 个字段，见下 |

`stats.data` 按顺序为：

```text
0 rate_Hz, 1 jitter_s, 2 drop_count, 3 non_monotonic_count,
4 acc_mean_x, 5 acc_mean_y, 6 acc_mean_z,
7 acc_std_x, 8 acc_std_y, 9 acc_std_z,
10 gyro_mean_x, 11 gyro_mean_y, 12 gyro_mean_z,
13 gyro_std_x, 14 gyro_std_y, 15 gyro_std_z,
16 latency_s
```

加速度均值/标准差统一为 g，角速度统一为 rad/s。`drop_count` 统计超过 `drop_factor/nominal_rate_hz` 的间隔事件，并非精确丢失消息数。`acc_unit_is_g=true` 对应设计中提供的驱动，连接硬件后须检查静止模长约为 1；如果驱动已转换为 m/s²，应设为 false。

`latency_s` 在实时模式为节点时钟减消息时间戳，仅在二者时钟基准一致时可解释为接收延迟，负值保留用于诊断。回放模式将此字段置为 NaN，避免误当作原始网络延迟。标准差使用总体方差，稳定的 Welford 更新避免大偏移数据的数值消减。

### 里程计分析节点

订阅 `/Odometry`，由位置差分计算世界系速度，累计里程、最大速度、距起点位移、姿态角，并比较同一时间戳 TF 与 Odometry 位姿。

| 输出 `/analysis/odom/…` | 类型 | 单位/含义 |
|---|---|---|
| `distance` | Float64 | m，跨样本阈值过滤后的累计路径长度 |
| `velocity` | TwistStamped | m/s，世界系差分线速度；角速度字段未计算，保持 0 |
| `speed` | Float64 | m/s，差分速度模长 |
| `rpy` | Vector3Stamped | 度，x/y/z 分别为 roll/pitch/yaw |
| `displacement` | Float64 | m，距本分析段起点的直线距离 |
| `tf_error` | Float64MultiArray | `[位置误差_m, 旋转误差_deg, 累计失败数]` |
| `markers` | MarkerArray | 按速度着色轨迹、里程标签、当前位置状态 |

TF 查询采用独立监听线程和非阻塞重试，最多等待 `tf_wait_s=0.2` s；超时发布 NaN 和递增失败计数。旋转误差归一化四元数后取绝对点积，兼容 q 与 -q。错误 frame、非法四元数和非有限输入被拒绝。

`min_step_m` 的参考位置保留到累计位移越过阈值，避免每帧不足 5 mm 的慢速移动被全部丢弃。这是基于位移的噪声过滤，不保证静止时里程绝不增长，也会丢失阈值内的来回运动。回到起点后的 `displacement` 反映起终点差异，不能在没有外部真值时当作严格定位精度。

轨迹最多保留 5000 点，超出后降采样；里程标签最多保留 200 个，使用循环 ID 覆盖旧标签。轨迹、里程、状态使用不同 namespace，避免 ID 冲突。

两个节点都忽略重复时间戳；时间倒退时开始新统计/轨迹段，并清除旧窗口/轨迹，异常计数保留。仅位置系发生重置且时间仍递增时无法自动判断是不是新轨迹，应重启节点。参数在启动时读取，修改 YAML 后重启节点生效。

## 实机启动、录制与检查

先给雷达供电并连接以太网，确认主机网卡和雷达处于同一子网，再编辑本包的 `config/MID360_config.json`：

- `MID360.host_net_info` 中 cmd/push/point/imu 的 `*_ip` 都改为实际主机 IP。
- `lidar_configs[0].ip` 改为实际雷达 IP。
- 默认副本 `192.168.1.41` / `192.168.1.130` 来自 ZIP，不能视为你的设备地址；不要向 JSON 添加注释。

```bash
ros2 launch mid360_analysis bringup.launch.py
# 无显示器：rviz:=false；先打通驱动/FAST-LIO：with_analysis:=false
# 自定义网络配置：user_config_path:=/absolute/path/MID360_config.json
```

该 Launch 默认同时启动两个分析节点。RViz Fixed Frame 为 `camera_init`，显示 Grid、TF 名称、配准点云、Odometry、Path 和分析 markers。

### WSL 镜像网络下的已验证配置

本次测试中，Windows Hyper-V 入站规则放行设备 UDP 后，SDK 已能收数，但默认 CycloneDDS 配置仍出现 IMU 有消息、点云无消息。将 DDS 消息限制为 1400 B、分片限制为 1200 B 后，真实点云和 FAST-LIO 输出正常。所有参与启动、录包、检查和回放的终端使用相同环境：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$(ros2 pkg prefix --share mid360_analysis)/config/cyclonedds_wsl.xml"
export ROS_DOMAIN_ID=86
unset ROS_LOCALHOST_ONLY
```

该配置仅供同一个 WSL 实例内的 ROS 节点通信，使用 loopback 和单播发现；跨机器 ROS 通信需要另配接口与发现地址。它不改变 Livox SDK 的以太网接口，驱动 JSON 仍应填实际主机 IP。Windows 侧还需允许 MID360 的 UDP 到 WSL；参考 [微软 Hyper-V 防火墙说明](https://learn.microsoft.com/windows/security/operating-system-security/network-security/windows-firewall/hyper-v-firewall)，不要把关闭整个防火墙作为常规方案。

新终端中：

```bash
ros2 run mid360_analysis check_topics.sh /home/simuel/mid360/docs/evidence
ros2 run mid360_analysis record.sh /home/simuel/mid360/bags/run01
# 停止录制使用 Ctrl+C，等待 rosbag 完成 metadata 写入
ros2 bag info /home/simuel/mid360/bags/run01
```

建议先静止 10 s，再绕行 1–2 分钟，最后回到起点静止 5 s。录制 `/livox/lidar /livox/imu /Odometry /path /cloud_registered /tf /tf_static`。传感器/里程计/TF 必须实际有数据，不能只检查话题名称。

检查脚本会记录每条命令与退出码，将 Topic 输出保存为文本、TF 文件保存到独立目录。`topic hz` 和 `tf2_echo` 是持续命令，到达采样超时产生 exit=124 属于预期；其他命令超时则需检查输入是否存在。

## 离线回放

停止实机 Launch 后：

```bash
ros2 launch mid360_analysis analysis.launch.py bag:=/home/simuel/mid360/bags/run01
# 无显示器：rviz:=false；回放速度：rate:=0.5
```

回放只消费已记录的 FAST-LIO 输出，不启动驱动或重新运行 FAST-LIO。分析、RViz 使用 `use_sim_time=true`；回放发布 `/clock` 并默认延迟 2 s 留出发现时间。仅回放关键输入，防止旧 `/analysis/*` 与本次分析结果混合。`/tf_static` 强制使用 transient_local，以支持稍晚加入的 RViz/TF 订阅者。

也可拆开运行：

```bash
# 终端 A
ros2 launch mid360_analysis analysis.launch.py rviz:=false
# 终端 B
ros2 run mid360_analysis play.sh /home/simuel/mid360/bags/run01 1.0
```

`analysis.launch.py` 默认不补发静态 TF，因为标准录包已包含 `/tf_static`。只有 bag 缺少 `body → livox_frame` 时才添加 `with_static_tf:=true`，并提供与录制时一致的 `fastlio_config`。

## 验证结果

```bash
ros2 topic list -t
ros2 topic echo --once /analysis/imu/stats
ros2 topic echo --once /analysis/imu/is_static
ros2 topic echo --once /analysis/odom/tf_error
ros2 topic echo --once /analysis/odom/displacement
# 可选安装 ros-humble-rqt-plot 后：
rqt_plot /analysis/odom/speed/data /analysis/imu/acc_norm/data
```

- IMU 频率应接近实际标称值，点云频率应接近配置值；用实测更新文档。
- 静止完整窗口内 `is_static=true`、`acc_norm≈1 g`、速度接近 0；检查里程是否仍被噪声累积。
- TF 可用时误差应接近 0；NaN 表示没有有效比较结果，必须同时检查失败计数。
- 检查慢速移动和回到起点的结果，不能只录制静止数据。
- 合成自动化测试验证软件逻辑，不能证明真实网络、单位、外参、建图或定位精度正确。

运行证据分别放到 `docs/images/`、`docs/evidence/` 和 `docs/videos/`。超过 50 MB 的视频使用外链，bag 不提交 Git。提交按核心统计、ROS2 节点与启动配置、文档与验收说明组织，保留至少 3 条有意义的历史。

## AI 使用情况

基础 C++ 节点、统计工具、Launch、配置、脚本、测试和 README 由 Codex 辅助实现；已针对提供的 ZIP 核对接口，完成自动化验证及 2026-10-06 的真实 MID360 链路测试。原始录包与检查结果保留本地；尚未完成的路线、静止零偏、定位精度及驱动正常退出验收见验证记录。
