# AXERA VIN 标定接口、vis 映射与 ABI 约束

## 权威 SDK

开发基线：

```text
D:\tool\mipi_tools\sensor_timing_check_tool\bsp_sdk\include
```

程序固定记录并核验 `ax_base_type.h`、`ax_global_type.h`、`ax_vin_api.h`、`ax_isp_common.h`、`ax_ivps_api.h`、`ax_ivps_type.h`、`ax_sys_api.h` 和 `ax_pool_type.h` 的 SHA-256。VIN ABI 以最新 `bsp_sdk/include` 为权威，运行时实际哈希会写入会话元数据。

`AX_VIDEO_FRAME_T` 必须满足：

| 项目 | 值 |
|---|---:|
| 基础结构大小 | 232 bytes |
| `u64PTS` 偏移 | 192 |
| `u64SeqNum` 偏移 | 200 |
| 颜色分量数 | 3 |
| `AX_VIDEO_FRAME_INFO_T` 大小 | 240 bytes |
| `AX_IMG_INFO_T` 大小 | 576 bytes |
| `AX_IMG_INFO_T.tIspInfo` 偏移 | 240 |

ctypes 的独立 `AX_VIDEO_FRAME_T` 与完整 `AX_IMG_INFO_T` 都带 512 字节尾部保护区，但 SDK 字段偏移保持不变。VIN 获取后，程序保留完整的同一个 `AX_IMG_INFO_T` 所有权对象；不能只保留或回传其中的 `AX_VIDEO_FRAME_T`。若自动找到或显式传入 `--sdk-include-dir`，任何哈希、字段顺序或 VIN 函数签名差异都会阻止运行。板端没有头文件时仍执行固定 ABI 检查并产生质量警告，建议部署上述 8 个权威头文件进行严格验收。

## 通道和生命周期

标定采集的默认接口：

```text
左目：AX_VIN_GetYuvFrame(pipe=2, chn=0, ...)
右目：AX_VIN_GetYuvFrame(pipe=0, chn=0, ...)
格式：NV12
分辨率：跟随相机 config，脚本只核验 3840x2160 或 1920x1080
```

vis 模式另有两套已确认输出：VIN 仍为左 pipe 2、右 pipe 0、chn 0、NV12 3840×2160；IVPS 为左 grp 7、右 grp 8、chn 0、灰度 640×384。它们记录在配置中，但标定录制命令只读取上述全分辨率 VIN 路径。

`AX_SYS_Init` 和 `AX_VIN_Init` 只附着当前 Python 进程的 SDK/VB/VIN 消费者上下文。脚本不会创建、启动、停止或重新配置 VIN/IVPS pipe/group/channel。板端应设置 `BSP_MSP_DIR=/soc/lib` 和 `LD_LIBRARY_PATH=/soc/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}`；后者用于解析 `libax_proton.so` 的传递依赖。脚本同时先以全局符号方式加载 `libax_sys.so`；若同目录存在，则依次预加载 `libax_interpreter.so`、`libax_engine.so`，最后加载导出 VIN 接口的 `libax_proton.so`，作为环境遗漏时的容错。

每对图像采用以下严格生命周期：

1. 获取左帧和右帧。
2. 若 PTS 差超过 2000 µs，释放较早帧并重新获取，最多 10 次。
3. 校验分辨率、stride、压缩模式和像素格式。
4. 通过 `AX_SYS_MmapCache` 获取虚拟地址，并调用 `AX_SYS_MinvalidateCache`。
5. 将有效 Y 行直接复制到可复用的 Python 自有 `bytearray`；BMP 同时形成 bottom-up 行序，JPEG 保持 top-down，不再保留中间 raw/gray 两份整图。无 NumPy 时保留逐行拷贝回退路径。
6. 对脚本创建的映射立即调用 `AX_SYS_Munmap`。
7. 左右帧分别以获取时的完整 `AX_IMG_INFO_T`，按原 pipe/chn 调用一次 `AX_VIN_ReleaseYuvFrame`。
8. 在不再持有 SDK 映射或帧的情况下，由图像工作线程写 BMP 或编码 JPEG；全部相关任务结束后才能复用缓冲。

任何中途异常也执行相同的释放规则。

## 像素格式

底层灰度提取器支持以下未压缩格式，便于无硬件测试和明确拒绝未知布局：

- `AX_FORMAT_YUV400`
- `AX_FORMAT_YUV420_PLANAR` / `_VU`
- `AX_FORMAT_YUV420_SEMIPLANAR`（NV12）
- `AX_FORMAT_YUV420_SEMIPLANAR_VU`（NV21）
- `AX_FORMAT_RGB888`，按 SDK 规定的内存 BGR 顺序转灰度
- `AX_FORMAT_BGR888`，按 SDK 规定的内存 RGB 顺序转灰度

YUV 格式直接提取 Y 平面。生产标定路径在此基础上进一步强制源格式为 `AX_FORMAT_YUV420_SEMIPLANAR`（NV12）；若首帧或后续帧不是 NV12，会立即失败并保留 `.failed` 现场数据。压缩帧、10 bit 格式和未知格式也会明确报错。

## 4K/1080p

`--resolution 3840x2160` 和 `--resolution 1920x1080` 是被动契约。每对图像的实际 `u32Width/u32Height` 必须匹配；脚本不会调用 `AX_VIN_SetChnAttr`。切换分辨率应由相机 config/生产管线完成，并在录制前运行 `probe` 验证。
