# calib_capture 项目维护规范

本文件是项目唯一的 Agent 说明。修改代码前先阅读本文件，以及 `doc/README.md`、`doc/使用操作说明.md`、`doc/AXERA_IVPS.md` 和 `doc/DATASET_FORMAT.md`。

## 项目范围

本项目以 Python 采集双目标定及双目–IMU 外参标定数据：

- 双目通过 `ctypes` 调用 AXERA VIN 接口；
- IMU 通过 ROS 2 `sensor_msgs/msg/Imu` 订阅；
- 目录数据集是原始数据，ROS1 bag 仅在采集完成后离线生成；
- 不使用 `ros2 bag record`，也不负责启动、停止或重配相机流水线。

关键路径：

- 本地工程：`Q:\project\kalibr_cedarobo`
- 脚本：`Q:\project\kalibr_cedarobo\script`
- SDK 权威头文件：`D:\tool\mipi_tools\sensor_timing_check_tool\bsp_sdk\include`
- 板端宿主机：`/workspace/test/Vincent/calib_capture`
- AX650 Docker：`/workspace/minsky/Vincent/calib_capture`

外部 SDK 和参考工程只读，不得修改。

## 硬件映射

| 模式 | 输出 | 左目 | 右目 | 接口 | 格式与分辨率 |
|---|---|---|---|---|---|
| cali | VIN | pipe 2 / chn 0 | pipe 0 / chn 0 | `AX_VIN_GetYuvFrame` | NV12；3840×2160 或 1920×1080 |
| vis | VIN | pipe 2 / chn 0 | pipe 0 / chn 0 | `AX_VIN_GetYuvFrame` | NV12，3840×2160 |
| vis | IVPS | grp 7 / chn 0 | grp 8 / chn 0 | `AX_IVPS_GetChnFrame` | YUV400，640×384 |

正式标定只使用 cali VIN。不得将 vis 的 640×384 IVPS 图像用于标定录制。`--resolution` 只校验上游实际输出，不得修改相机 config、重配 VIN 或缩放图像。

## 采集契约

- `cam0` 始终是左目，`cam1` 始终是右目。
- 默认左目 pipe 2/chn 0、右目 pipe 0/chn 0，CLI 可显式覆盖。
- 标定输入必须为 NV12，落盘图像为无损 `mono8` BMP。
- 数据文件不计算 SHA-256，录制、结束收尾、离线校验和 bag 导出都只做结构与时序检查。BMP 按原 bottom-up 布局大块写入；不生成 `meta/checksums.sha256`。SDK 启动契约哈希不属于逐帧数据处理，仍保留。
- 使用 `u64PhyAddr[0]` 调用 `AX_SYS_MmapCache`；缓存失效后复制 Y 平面到 Python 内存，再立即 Munmap 并释放 SDK 帧。不得解引用其他进程的 `u64VirAddr`。
- `AX_VIN_GetYuvFrame` 获取完整 `AX_IMG_INFO_T`，必须把同一个对象交回原 pipe/chn 的 `AX_VIN_ReleaseYuvFrame`。每个成功获取的帧只释放一次。
- 左右 PTS 差默认不超过 2000 µs；超限时仅重取较早侧，最多 10 次。
- 相机时间戳使用各自 `u64PTS × 1000` ns；IMU 使用原始 `header.stamp`。
- 相机与 IMU 首帧差超过 5 秒时拒绝 `camera-imu` 录制。
- 连续采集按完整双目对限频、分频和入队；任何丢弃都必须同时丢弃左右目。
- `--save-rate-hz` 与 `--frame-divisor N` 仅用于连续双目和 `camera-imu`，两者互斥。默认按 PTS 保存双目 5 Hz（约 200 ms 一对）；可显式使用源序号分频。IMU 收到的消息全部保存。
- PTS 限频以首对时间锚定目标时刻，选择到达目标的第一对，避免周期舍入累计导致边界多跳一帧。
- 采集先写 `.inprogress`，成功校验后提交正式目录；失败时保留为 `.failed`。

## 输出目录

录制命令的 `--output` 是输出根目录，默认 `./data`。每次录制自动生成：

```text
<output>/YYYY-MM-DD_HH-MM-SS/
```

目录名使用运行设备本地时间。同一秒内重复启动且目录已存在时必须报错，不得覆盖旧数据。实际会话目录必须写入命令输出和 `meta/session.json`。

## ABI 基线

权威 SDK 当前约束：

- `AX_VIDEO_FRAME_T`：232 字节；
- `u64PTS` 偏移：192；
- `u64SeqNum` 偏移：200；
- `AX_VIDEO_FRAME_INFO_T`：240 字节；
- `AX_IMG_INFO_T`：576 字节；
- `AX_IMG_INFO_T.tIspInfo` 偏移：240；
- ctypes 容器保留 512 字节尾部保护区。

运行库默认位于 `/soc/lib`。板端运行前必须设置 `BSP_MSP_DIR=/soc/lib`，并把 `/soc/lib` 放入 `LD_LIBRARY_PATH`，以便动态加载器解析 `libax_proton.so` 对 `libax_engine.so` 等库的传递依赖。脚本仍按 `libax_sys.so`、`libax_interpreter.so`、`libax_engine.so`、`libax_proton.so` 顺序加载现有库作为容错。初始化顺序为 `AX_SYS_Init`、`AX_VIN_Init`，退出时反向释放。SDK 头文件更新后，必须重新核对结构、函数签名及 SHA-256，不能绕过 ABI 或哈希检查。

## 配置和数据规则

- 配置优先级：命令行 > INI `[capture]` 配置 > 内置默认值。`--config` 使用 Python 标准库 `configparser` 读取 UTF-8（可带 BOM）文件，不依赖 PyYAML；模板为 `script/default_config.ini`，未指定 `--config` 时不自动加载。
- INI 只允许 `[capture]` 段，不接受 `[DEFAULT]` 参数、其他段、重复键或未知键。值不加引号；注释单独以 `;` 或 `#` 起行。关闭插值，`%`、`${...}` 按字面处理。cali/vis 硬件清单只记录在模板顶端注释中，不设置 `hardware_layout` 段。
- 旧的自定义 YAML/JSON 配置需转换内容，不能只改扩展名。目录数据契约中的 `dataset.yaml` 和 JSON 元数据保持不变；`dataset.yaml` 写入与校验不依赖 PyYAML。
- 新配置项必须同步修改默认配置、数据类、CLI、会话元数据、文档和测试。
- 不得静默改变左右目含义、时间单位、目录 schema 或默认话题。
- 当前数据集为 schema 2，标记 `data_integrity: structural`；读取时兼容 schema 1，忽略旧数据的校验和清单。同长度像素变化不再由校验器检测。
- `meta/session.json` 的 `write_performance` 记录每对 BMP 与 CSV 提交的平均、最大及累计耗时；不包括取帧、排队等待或 `fsync`。
- 仅 `camera-imu` 和 `probe --with-imu` 可以依赖 ROS 2 Python 环境。
- 写盘队列满时只丢弃完整双目对，并记录 AX 错误、源序号缺口、分频/限频跳过数及队列丢弃数。
- 不做磁盘容量或写速硬准入；4K 数据量只提示。

## 测试要求

从 `script` 目录执行：

```bash
python3 calibration_data.py self-test
python3 -m unittest discover -s tests -v
```

涉及 ABI、映射、同步、释放、分频或数据格式时，必须覆盖对应无硬件测试。安装 `rosbags==0.11.0` 后，ROS1 bag 生成与回读测试不得跳过。没有执行的可选依赖测试或板端测试必须明确说明。

板端验收至少检查：

- 4K 与 1080p 分别执行 `probe`；
- 两侧为 NV12，尺寸与参数一致；
- PTS 严格递增，源帧率合理，左右差不超过 2 ms；
- IMU 接近预期频率且与相机具有共同时间窗口；
- 录制期间无其他消费者竞争相同 VIN 输出。

## 板端访问

除非用户在当前任务明确授权，不得执行 SSH、SCP、`docker exec`、部署或板端运行。停止进程、切换相机模式或修改 Docker/系统配置需要单独授权。

获得授权后的入口：

```text
wsl.exe -d Ubuntu-22.04
ssh root@192.168.8.100
docker exec -it ax650_aarch64v1.1_test bash
cd /workspace/minsky/Vincent/calib_capture
export BSP_MSP_DIR=/soc/lib
export LD_LIBRARY_PATH=/soc/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
export ROS_DOMAIN_ID=83
source /opt/ros/humble/setup.bash
```

宿主机 `/workspace/test/Vincent` 映射到容器 `/workspace/minsky/Vincent`。板端只同步 `script/`；不要同步 `doc/`、顶层 `AGENTS.md`、`data/` 或 `tmp/`，也不要向容器映射路径重复复制。板端使用现有环境，不安装新包或库；`requirements-capture.txt` 中的 NumPy 仅用于可选加速。ROS1 bag 导出和 `rosbags` 依赖只在 PC 本机使用。
