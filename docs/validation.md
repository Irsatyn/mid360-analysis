# 基础代码验证记录

验证日期：2026-10-06（Asia/Shanghai）。本记录属于软件验证，不是实机运行证据。

## 已完成

- `colcon build --symlink-install --packages-select mid360_analysis`：成功构建两个 C++17 节点。
- `colcon test --packages-select mid360_analysis --return-code-on-test-failure`：成功。
- `colcon test-result --verbose`：35 tests，0 errors，0 failures，0 skipped。此汇总包含 ament 的测试组结果；实际测试用例为 13 项 C++ 测试和 18 项 Python 测试。
- Python 测试：6 项真实 DDS 节点集成、11 项 Launch 配置/旋转校验、1 项完整 analysis Launch 合成 rosbag 回放。
- 合成 bag 回放验证：`use_sim_time`、IMU 静止检测与约 200 Hz、回放延迟字段 NaN、累计里程 0.12 m、速度 0.1 m/s、延迟 TF 比较结果零误差与零失败。
- 两个主 Launch 的 `--show-args`、Python AST、Bash 语法、XML/JSON/YAML/RViz 配置解析及脚本参数检查通过。

构建期间 ROS Humble 自带 `gtest_vendor` 的旧 CMake 最低版本产生弃用警告；未观察到本项目 C++ 编译警告。没有修改系统或第三方源码来压制此警告。

## 尚需实机完成

当前环境未安装 Livox-SDK2、livox_ros_driver2 和 fast_lio；因此未验证第三方包完整构建、驱动收数和建图。设备 IP、加速度实际单位、实际频率、外参、时间基准均需要连接雷达确认。

尚未录制真实 MID360 bag，没有真实 RViz 截图/视频。README 中提供了完整准备、启动、检查、录制、回放和验收步骤。
