# xr-device-io

共享相机/IMU、原始手柄传输、时间戳和缓冲所有权。


构建入口和依赖见下方；工程 Wiki/Blog 与跨仓库规划集中在 [openkylin-xr-docs](https://gitee.com/bigbits/openkylin-xr-docs)。

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
