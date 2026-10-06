# 驱动退出崩溃修复

## 原因

提供的驱动正常收数后，在单次 SIGINT 停止时出现 SIGSEGV 或 `double free`。真实硬件回归用例在修复前收到 640 条 IMU、55 帧点云，但退出码为 -11。

AddressSanitizer 调用栈显示：Livox-SDK2 的 `std::shared_ptr<spdlog::logger>` 清理调用了系统 `libspdlog.so.1` 的 `_M_dispose()`，随后发生非法内存访问。SDK 内置 spdlog 1.3.1，系统版本为 1.9.2；SDK 导出的同名 C++ 符号发生了版本混用。错误并非仅由重复 SIGINT 引起。

## 修复方法

用 `tools/build_livox_sdk_isolated.py` 构建本地 SDK。链接器导出表仅保留 SDK 头文件声明的 63 个公共 C API，其余符号局部化，防止 SDK 的内部 spdlog 与 ROS 的 spdlog 相互覆盖。第三方源码和 `/usr/local` 系统库均不修改。

从本仓库根目录执行：

```bash
python3 tools/build_livox_sdk_isolated.py \
  --source /absolute/path/Livox-SDK2 \
  --prefix "$PWD/livox_ros_driver2_local_sdk"

# 在原驱动工作空间中重新链接；两个路径均使用绝对路径。
source /opt/ros/humble/setup.bash
colcon build --packages-select livox_ros_driver2 --cmake-args \
  -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
  -DLIVOX_LIDAR_SDK_LIBRARY=/home/simuel/mid360/livox_ros_driver2_local_sdk/lib/liblivox_lidar_sdk_shared.so \
  -DLIVOX_LIDAR_SDK_INCLUDE_DIR=/home/simuel/mid360/livox_ros_driver2_local_sdk/include \
  -DCMAKE_INSTALL_RPATH=/home/simuel/mid360/livox_ros_driver2_local_sdk/lib
```

也可先不重新链接，在启动驱动的终端把该本地 SDK 的 `lib` 目录放到 `LD_LIBRARY_PATH` 最前。不需要对整个系统设置 `LD_PRELOAD`。

生成的 SDK 与构建目录位于 Git 忽略目录，构建工具可重复使用。工具检查实际导出的符号集合与公共 API 一致，缺失 API 或泄漏其他符号会报错。

## 针对性回归

先 source ROS 和驱动工作空间，使用 README 中已验证的 WSL DDS 环境，并确认没有其他驱动实例：

```bash
python3 tools/check_driver_shutdown.py \
  --driver /absolute/path/workspace/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node \
  --config /absolute/path/MID360_config.json \
  --log-dir bags/driver_shutdown_fix/check --cycles 3
```

用例要求每次都实际收到 IMU 和 CustomMsg 点云，然后只给驱动发送一次 SIGINT，要求退出码 0 且未强制结束；Launch 父进程返回 0 不能代替这个检查。日志可能含设备标识，应保留在 `bags/` 下。

初次修复验证：连续三次真实收数与停止均通过，每次收到 55 帧点云、628–637 条 IMU，退出码均为 0。该回归针对退出崩溃，不代表静止零偏或定位精度验收。

同一个 ASan 诊断驱动换用隔离 SDK 后退出码为 0，未再出现原来的非法内存访问（此次关闭了泄漏检测）。最终完整 bringup 复测核对了驱动进程的 `/proc/<pid>/maps`，确认实际加载的是本地隔离 SDK；收到 80 帧点云、79 条里程计及配准点云，79 对 TF / Odometry 误差为零。停止时驱动、FAST-LIO、两个分析节点和静态 TF 五个进程全部正常退出。最终软件复测的 colcon 汇总为 50 tests，0 errors、0 failures、0 skipped。

原始诊断与回归日志位于本地 `bags/driver_shutdown_fix/`。2026-10-06 最终复测连续三次分别收到 605/656/592 条 IMU、各 55 帧点云，均正常退出；可提交结果见 [最终退出复测](evidence/final_20261006/driver_shutdown.json)。
