# 基础代码验证记录

验证日期：2026-10-06（Asia/Shanghai）。软件验证和真实 MID360 测试分别记录如下。

## 已完成

- `colcon build --symlink-install --packages-select mid360_analysis`：成功构建两个 C++17 节点。
- `colcon test --packages-select mid360_analysis --return-code-on-test-failure`：成功。
- `colcon test-result --verbose`：50 tests，0 errors，0 failures，0 skipped。此汇总包含 ament 的测试组结果；实际测试用例为 13 项 C++ 测试和 33 项 Python 测试。
- Python 测试：6 项真实 DDS 节点集成、26 项 Launch 配置/旋转/网络校验、1 项完整 analysis Launch 合成 rosbag 回放。回归测试验证 FAST-LIO 子 Launch 的 `rviz=false` 不覆盖外层参数，以及 IP 覆盖/路由选择、自定义配置保留、临时文件清理和 DDS 配置生成。
- 合成 bag 回放验证：`use_sim_time`、IMU 静止检测与约 200 Hz、回放延迟字段 NaN、累计里程 0.12 m、速度 0.1 m/s、延迟 TF 比较结果零误差与零失败。
- 两个主 Launch 的 `--show-args`、Python AST、Bash 语法、XML/JSON/YAML/RViz 配置解析及脚本参数检查通过。

构建期间 ROS Humble 自带 `gtest_vendor` 的旧 CMake 最低版本产生弃用警告；未观察到本项目 C++ 编译警告。没有修改系统或第三方源码来压制此警告。

## 真实 MID360 链路测试

环境为 Ubuntu 22.04 / ROS2 Humble / WSL 镜像网络。已安装 Livox-SDK2，并在独立临时工作空间成功编译提供 ZIP 中的 livox_ros_driver2、FAST-LIO，第三方源码未修改。实测雷达直连 ping 3/3 成功；Windows Hyper-V 对设备来源 UDP 放行后，SDK 的 8 秒采样收到 12350 个点云包及 1186 个 IMU 包。

默认 DDS 配置下驱动调用点云发布成功，但订阅端收不到大消息。使用 `config/cyclonedds_wsl.xml` 的 1400 B 消息上限和 1200 B 分片后，驱动、FAST-LIO、两个分析节点及 RViz 的实时链路运行成功；配置用法见 README。该结果证明本次配置下链路可用，尚未独立确认默认大包失败的底层原因。

22:46:04–22:46:34 录制的真实 bag 共 30.229 秒、126.9 MiB、7293 条消息：

| 话题 | 消息数 |
|---|---:|
| `/livox/imu` | 6053 |
| `/livox/lidar` | 302 |
| `/Odometry` | 302 |
| `/cloud_registered` | 303 |
| `/tf` | 302 |
| `/path` | 30 |
| `/tf_static` | 1 |

直接反序列化 bag 检查：IMU 时间戳频率 200.162 Hz，加速度模长均值 0.99265 g，确认 `acc_unit_is_g=true`；点云时间戳频率 10.0001 Hz，平均每帧 20016 点。IMU 时间戳无倒退或重复，6 个间隔超过 7.5 ms，最大间隔 13.644 ms；这些是间隔异常事件，不能直接换算为丢失消息数。

实时探针收到 299 对相同时间戳的 Odometry / TF，位置及旋转误差均为零，分析节点 TF 失败数为零。TF 链为 `camera_init → body → livox_frame`。Python 探针同时订阅大点云时其 best-effort IMU 订阅有丢样，因此以上 IMU 频率采用完整 bag 检查结果。

停止实机链路后，使用本项目 `analysis.launch.py` 回放该真实 bag：点云 302 帧、里程计及 TF 各 302 条、配准点云 303 帧全部收到；302 对 TF / Odometry 误差为零，分析 TF 失败数为零。两个分析节点 `use_sim_time=true`，IMU 回放延迟字段为 NaN。回放结束累计里程 1.07073 m、起终点位移 0.08623 m，这些只是本段估计输出，没有外部真值，不作为定位精度指标。

原始 bag、探针 JSON、设备日志及本地报告保存于 `bags/hardware_test_20261006/`，按 `.gitignore` 保留本地，未上传到 public 仓库。首次仅有 IMU 的 `live_bag` 是排障产物；有效完整录制为 `full_chain_bag`。

## 验证边界

网络配置通用化后的本机复测：使用地址留空的 JSON 模板，仅指定 `lidar_ip`，实际按路由自动选择主机地址；通用 `cyclonedds.xml` 收到 80 帧点云、79 条里程计及配准点云，79 对 TF / Odometry 误差为零，五个节点正常退出，临时 JSON 已清理。原始记录保留在 `bags/network_config_test/`。目前没有第二台 ROS 主机，尚未验证双机通信；跨机器的路由、接口选择及防火墙仍需在实际网络中检查。

另用生成工具指定本机网卡 IPv4、禁用组播并添加单播 peer，在本机独立驱动/订阅进程之间收到 35 帧真实点云和 350 条 IMU，停止退出码为 0；验证了生成的配置被 Humble 的 CycloneDDS 接受，但这仍属于单机验证。

- 本次未完成规定路线、回到起点或外参精度验收，没有外部定位真值。
- 静止判定在本次实测中为 false；角速度不满足默认阈值，未人为放宽阈值，真实静止零偏估计尚未验收。
- 早期 WSLg 自动截图为黑图；最终通过 Windows `PrintWindow` 取得有效实机/回放截图与短视频，已提交到 `docs/images/` 和 `docs/videos/`。
- 提供的 FAST-LIO 发布的 `/path` 顶层 header 时间戳保持不变；里程计、点云及动态 TF 时间戳正常，路径时间分析应检查各 PoseStamped。
- 首次实机测试发现驱动退出时出现 `double free`。后续用 ASan 定位为 Livox-SDK2 内置 spdlog 1.3.1 与 ROS 系统 spdlog 1.9.2 的符号冲突；隔离 SDK 内部符号后，三次真实收数/停止、ASan 复测和最终五个节点的完整停止均通过。第三方源码未修改，详见 [驱动退出修复](driver_shutdown_fix.md)。静止零偏和定位精度验收仍待完成。

## 最终提交前复测

完整构建与测试再次通过：50 tests，0 errors，0 failures，0 skipped。首轮沙箱禁止 UDP 接口枚举；允许本机 DDS 网络后，完整测试通过，没有跳过测试。结果见 [软件测试报告](evidence/final_20261006/software_tests.txt)。

通用 DDS 配置下，驱动连续三次收到 605/656/592 条 IMU 和各 55 帧点云，均退出码 0，无强制结束。完整实机链路收到 199 帧点云、199 条里程计和配准点云，199 对 TF/Odometry 比较误差为零，分析 TF 失败数为零，连同 RViz 的 6 个进程全部正常退出。

真实 bag 回放探针在播放器开始后加入，收到 295 对 TF/Odometry，误差及失败计数为零，最终里程 1.07073 m、位移 0.08623 m；播放器、两个分析节点、RViz 共 4 个进程正常退出。两个分析节点的 `use_sim_time` 均查询为 true。295 是本次探针观察数量，原 bag 中共有 302 条 Odometry；多话题 best-effort IMU 探针丢样不能视为雷达设备丢样。

已补充真实 RViz 实机/回放截图、视频与 view_frames 图。Windows 屏幕坐标采集曾出现 DPI/遮挡偏差，最终改用指定窗口的 `PrintWindow`，经人工检查实际图像有效后才纳入仓库。相机距离从约 217 m 改为 12 m，并验证点云与分析 markers 可见。需求对应、最终文件范围及实际节点输出证据见 [最终核对](final_acceptance.md)。原始日志保留在 `bags/final_validation_20261006/`。
