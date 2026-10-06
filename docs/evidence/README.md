# 运行证据状态

2026-10-06 已完成真实 MID360 实时链路、录包及离线分析回放，结果见 [验证记录](../validation.md)。原始 bag、设备日志和探针 JSON 保存在本地忽略目录 `bags/hardware_test_20261006/`，未上传到 public 仓库。RViz 已实际启动，但 WSLg 自动截图为黑图，不能作为显示证据。

图像和路线验收仍需补充：

- `docs/images/rviz_live.png`：实机点云、Odometry、TF。
- `docs/images/tf_tree.png`：TF 树，并保留 view_frames 生成文件。
- `docs/images/topic_info.png`：话题类型、QoS、实测频率。
- `docs/images/bag_info.png`：自行录制 bag 的 ros2 bag info。
- `docs/images/rviz_replay.png`：离线回放及分析 markers。
- `docs/images/analysis_echo.png`：分析话题输出/曲线。
- `docs/videos/`：实机、回放、自写节点运行短视频；大文件使用 README 外链。

`ros2 run mid360_analysis check_topics.sh <本目录>` 会生成 topics 文本和独立 frames 子目录。
自动化测试结果和合成 bag 仅属于软件验证，不应标注为真实雷达运行证据。
