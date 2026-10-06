# 最终测试与提交核对

日期：2026-10-06（Asia/Shanghai）。依据根目录 `需求.md` 核对。完整软件测试、真实 MID360 链路、真实 bag 离线回放均在本次提交前重新运行。

## 测试结果

- 分析包构建成功；colcon 汇总 **50 tests，0 errors，0 failures，0 skipped**。实际为 13 项 C++、6 项 DDS 集成、26 项 Launch/网络配置、1 项合成 bag 回放测试，另有 4 项测试组汇总。见 [测试报告](evidence/final_20261006/software_tests.txt)。
- 首轮沙箱禁止 UDP 接口枚举，DDS 测试不能创建节点；允许本机 DDS 通信后完整测试通过。没有跳过失败测试或修改断言。
- 真实驱动连续三次收到 IMU / 点云后，仅发送一次 SIGINT，退出码均为 0，无强制结束。三次 IMU 605/656/592 条、点云各 55 帧。见 [退出复测](evidence/final_20261006/driver_shutdown.json)。
- 完整实机链路收到 199 帧点云、199 条 Odometry 和配准点云，199 对同时间戳 TF/Odometry 位置和旋转误差均为零，TF 分析失败计数为零；驱动、FAST-LIO、两个分析节点、静态 TF、RViz 共 6 个进程正常退出。见 [实机结果](evidence/final_20261006/live_result.json)。
- 真实 bag 离线回放收到 295 对 TF/Odometry，比较误差及失败计数为零，两个分析节点 `use_sim_time=true`；播放器、两个分析节点、RViz 共 4 个进程正常退出。探针在播放器开始后加入，295 不是 bag 总数；原 bag 有 302 条 Odometry。见 [回放结果](evidence/final_20261006/replay_result.json)。
- Python AST、Bash 语法及帮助参数、JSON/XML/YAML/RViz 配置解析、两个 Launch 参数展开均通过。RViz 默认相机距离改为 12 m，并已用实机和回放检查点云及 markers 可见。

## 需求对应

| 需求 | 提交内容 / 验证证据 |
|---|---|
| 启动 MID360 Driver 与指定 FAST-LIO | `bringup.launch.py`；实机链路和正常退出复测 |
| LiDAR、IMU、Odometry Topic 信息 | README 数据流与 Topic 表；[Topic 列表](evidence/final_20261006/live_topics.txt)、[LiDAR QoS](evidence/final_20261006/live_lidar_info.txt)、[IMU QoS](evidence/final_20261006/live_imu_info.txt)、[Odom QoS](evidence/final_20261006/live_odom_info.txt) |
| RViz 观察传感器、Odometry、TF | [实机截图](images/rviz_live.png)、[实机短视频](videos/rviz_live.mp4)，固定坐标系 `camera_init` |
| TF 检查和说明 | README Frame/外参说明；[view_frames 原始图](evidence/final_20261006/tf_tree.pdf)、[TF 图](images/tf_tree.png) |
| 自行录制关键数据 rosbag | [bag 信息](evidence/final_20261006/bag_info.txt)，7 个关键话题、7293 条消息、30.229 s；原始 bag 保留本地 |
| 离线回放 | `analysis.launch.py`；[回放截图](images/rviz_replay.png)、[回放短视频](videos/rviz_replay.mp4)、回放探针结果 |
| 至少两个 ROS2 C++ 节点及新 Topic | `imu_stats_node.cpp`、`odom_analyzer_node.cpp`；全部 13 个分析话题的集成测试及真实回放监视器 |
| 自写节点运行截图及短视频 | [分析输出截图](images/analysis_echo.png)、[分析输出短视频](videos/analysis_topics.mp4)、[实际接收数据](evidence/final_20261006/analysis_topics.json) |
| ROS2 Package、Launch、配置、README | `mid360_analysis/`、根目录 README；构建、配置解析和 Launch 测试通过 |
| 至少 3 条有意义 Commit | 提交前已有 4 条独立历史；最终提交追加网络配置、SDK 隔离工具及最终运行证据 |
| AI 使用说明 | README 的 AI 使用情况章节 |

截图与视频来自真实进程。RViz 画面通过 Windows `PrintWindow` 读取指定 RViz 窗口，避免 WSLg 黑图与屏幕坐标偏差；分析输出画面由临时 Qt 监视器订阅正在运行的 C++ 节点生成，没有填入预设样本，不是 rqt。NaN 在可提交 JSON 中写为 `null`，监视器显示为 `None`。监视器采集的是回放中的片段，13 个话题各收到 225 条消息，其最后值不是完整 bag 的最终指标。

## 文件整理与交付范围

- 源码、通用配置、工具、说明、测试摘要、有效截图及短视频纳入 Git。
- 原始录包、设备标识日志、本地 SDK 和构建产物保留在忽略目录；根目录旧节点日志归档到 `log/final_validation/previous_node_logs/`。
- `.vscode/` 本机编辑器设置加入忽略规则；第三方 ZIP/源码不进入提交。
- 本次完成本地 commit；GitHub 仓库为 <https://github.com/Irsatyn/mid360-analysis>。远端需要包含最终 commit 才是完整交付，此次未执行 push。

## 验证边界

实测静止判定仍为 false，未放宽角速度阈值；真实静止零偏、规定路线/回到起点、外参精度和定位精度尚未验收。没有第二台 ROS 主机，双机 DDS 通信未验证。这些状态不由软件测试通过代替。

多话题 Python 探针采用 best-effort QoS，IMU 有订阅丢样；其接收频率不能当作设备频率。原始 bag 中 IMU 时间戳频率为 200.162 Hz，点云约 10 Hz。完整数据指标和早期排障过程见 [验证记录](validation.md)。
