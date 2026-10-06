# 运行证据

日期：2026-10-06（Asia/Shanghai）。最终软件与实机复测结果见 [最终核对](../final_acceptance.md)，历史测试过程和验证边界见 [验证记录](../validation.md)。

| 证据 | 文件 |
|---|---|
| 完整软件测试 | [测试报告](final_20261006/software_tests.txt)，50 tests / 0 errors / 0 failures / 0 skipped |
| 实机 Topic / QoS | [列表](final_20261006/live_topics.txt)、[LiDAR](final_20261006/live_lidar_info.txt)、[IMU](final_20261006/live_imu_info.txt)、[Odometry](final_20261006/live_odom_info.txt) |
| TF 树 | [PNG](../images/tf_tree.png)、[view_frames PDF](final_20261006/tf_tree.pdf)、[原始 Graphviz](final_20261006/tf_tree.gv) |
| 真实 rosbag 录制 | [ros2 bag info](final_20261006/bag_info.txt)，30.229 秒、7 个话题、7293 条消息 |
| 实机链路 | [探针结果](final_20261006/live_result.json)、[RViz 截图](../images/rviz_live.png)、[短视频](../videos/rviz_live.mp4) |
| 离线回放 | [探针结果](final_20261006/replay_result.json)、[RViz 截图](../images/rviz_replay.png)、[短视频](../videos/rviz_replay.mp4) |
| 回放时间配置 | [IMU use_sim_time](final_20261006/replay_imu_use_sim_time.txt)、[Odom use_sim_time](final_20261006/replay_odom_use_sim_time.txt) |
| 两个 C++ 节点实际输出 | [监视器截图](../images/analysis_echo.png)、[短视频](../videos/analysis_topics.mp4)、[接收数量和最后值](final_20261006/analysis_topics.json) |
| 驱动三次正常退出 | [检查结果](final_20261006/driver_shutdown.json) |

RViz 图像由 Windows `PrintWindow` 读取指定 RViz 窗口；早期 WSLg 黑图与屏幕坐标偏差采集不作为证据。分析画面是临时 Qt 监视器对真实回放节点的 13 个分析话题进行订阅后显示的输出，没有使用预设数据。JSON 中 NaN 写为 `null`；回放延迟和未就绪的静止零偏均可能没有有效值。

本目录仅保留精简的运行证据；Topic QoS 中与解释无关的运行时 GID 已移除。原始完整录包和设备日志保留在本地 `bags/`，构建、SDK、编辑器设置等本机产物不提交 Git。合成 bag 及自动化测试不能替代真实硬件证据。

静止零偏、规定路线/回到起点、外参/定位精度与双机 DDS 通信尚未验收，详见最终核对的验证边界。
