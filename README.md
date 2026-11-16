# xr-device-io

共享相机/IMU、原始手柄传输、时间戳和缓冲所有权。

IMU 使用原厂 SEE direct 路径：两个共享 DMA 通道、1250 微秒请求周期
（名义 800 Hz）、Android 未校准加速度/陀螺仪事件。这个周期来自 QVR 的
VERY_FAST 请求及传感器 HAL 的周期表；配置中的频率声明和 SEE 能力表不替代它。
实际频率必须用返回样本的时间戳统计，构建成功不代表采样或通知已实机通过。

`libxr-see-low-latency.so` 作为 `xr-hexagonrpcd -s -d slpi -P` 的私有插件运行。
会话管理器传入已发现的 `XR_SEE_ACCEL_SUID`、`XR_SEE_GYRO_SUID`
（各为小端 UID 两个整数的 `16位十六进制:16位十六进制` 文本），
`XR_SEE_IMU_SOCKET` 和显式启动确认 `XR_SEE_LOW_LATENCY=1`。
插件与已有 IMU 所有权锁互斥，只向同 UID 的一个 head 消费者发送记录；
安装不会自动启动它。FastRPC listener 和固件由会话管理器提供。
此插件要求 `xr-hexagonrpc-daemon >= 0.5.6` 的同步信号能力：listener 和
采集线程阻塞 SIGTERM/SIGINT，监督路径用 signalfd 接收后执行正常停流。
旧 daemon 会被显式拒绝，不能依靠 SA_RESTART 重发被中断的 RPC。

保留原厂 104 字节记录及计数器、时间戳和 bias 字段，通过
`sns_low_lat_stream_poll` 等待固件通知，禁止定时生成替代样本。
XSL2 包为 144 字节，包含完整原始记录和显式有符号 `qtimer_to_boottime_ns`。
QTimer 与 BOOTTIME 的偏移用计数器夹取测量；DSP 配置偏移固定为零，
统一 `decode()` 只做一次原始 QTimer 纳秒到 BOOTTIME 的转换，不修改原始记录。
挂起造成的时钟跳变结束会话，未来时间和顺序检查继续生效。
不再对未校准三轴值重复扣 bias，也不二次旋转 Android 坐标。
结束时先 stop 唤醒等待者，随后 join、deinit/unmap 和关闭独立 SDSP 会话。
通知、缓存可见性、真实频率和退出后的 DMA 回收仍需在目标硬件验证。


构建入口和依赖见下方；工程 Wiki/Blog 与跨仓库规划集中在 [openkylin-xr-docs](https://github.com/bitsk/openkylin-xr-docs)。

## 构建

相邻检出所需组件，并指定已准备的 xr-kernel 检出：

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc \
  -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ \
  -DXR_KERNEL_UAPI_DIR=/usr/include/xr-kernel
cmake --build build -j4
```

头部插件把统一采集作为库链接到唯一生产者，控制器和调试端消费其接口；这次不新增竞争采集进程。第三方 HexagonRPC 代码保留 COPYING，SDK/DSP 固件在设备运行时另行导入。

## openKylin 打包

本仓库包含独立的 `debian/` 配方。使用目标发行线的干净构建环境运行 `dpkg-buildpackage -us -uc -b`，预先准备 Build-Depends 和离线构建输入。包构建、目标系统安装和设备验收分别记录。

当前迁移不自动启用竞争采集、切换活动运行时或写设备启动分区；已有验证基线继续保留。

构建依赖通过已安装的 `xr-kernel-uapi-dev` 和 `libxr-device-dev` 提供；可用 CMAKE_PREFIX_PATH 选择开发包前缀。

## 共享相机输出

相机生产者在同一硬件锁内可选采集 IMX 彩透，并向原生消费者导出 QSP1 RAW10 快照；不为透视另开相机所有者。支持已配置的 1928×1088 与 1728×1728 裁剪。四路追踪预览在图像与 QCFT 元数据关联后发送 ACP2，保留有效性和曝光相位，缺少有效关联时保持未分类。手输入 tap 继续消费原来的已关联帧。
