# calib_capture 项目维护规范

## v1.0.0 统一契约

软件包、CLI、INI 与项目公开结构化文件统一版本 `1.0.0`；JSON/YAML 的
`schema_version` 使用字符串。数据集使用 `type: kalibr_directory_dataset`，
其他文档使用 `kind` 区分。只写和读新契约，不兼容旧整数 schema 或旧布局，
不得迁移、改名、覆盖或删除历史数据。ROS 消息、bag 和 SDK 属于第三方标准，
不修改其版本。采集与 Kalibr 两工程可在用户授权的联动任务中同步修改接口，
但不得以改写历史数据代替集成测试。

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
- 标定输入必须为 NV12，落盘默认无损 `mono8` BMP；可配置 `image_format = jpeg`、`jpeg_quality = 95` 保存有损灰度 JPG。质量范围 1–100；JPEG 只使用 OpenCV/NumPy，不使用 Pillow，不自动安装板端依赖，不缩放图像。
- 数据文件不计算 SHA-256，录制、结束收尾、离线校验和 bag 导出都只做结构与时序检查。BMP 按原 bottom-up 布局大块写入；不生成 `meta/checksums.sha256`。SDK 启动契约哈希不属于逐帧数据处理，仍保留。
- 使用 `u64PhyAddr[0]` 调用 `AX_SYS_MmapCache`；缓存失效后直接复制有效 Y 平面到 Python 自有缓冲区，再立即 Munmap 并释放 SDK 帧。BMP 在复制时形成 bottom-up 行序，JPEG 保持 top-down。不得解引用其他进程的 `u64VirAddr`，不得让 SDK 映射或帧进入编码线程。
- `AX_VIN_GetYuvFrame` 获取完整 `AX_IMG_INFO_T`，必须把同一个对象交回原 pipe/chn 的 `AX_VIN_ReleaseYuvFrame`。每个成功获取的帧只释放一次。
- 左右 PTS 差默认不超过 200 µs（含边界）；超限时仅重取较早侧，最多 10 次。
- 相机时间戳使用各自 `u64PTS × 1000` ns；IMU 使用原始 `header.stamp`。
- `camera-imu` 默认在图像窗口前、后各保存至少 1 秒 IMU，分别由 `imu_preroll_s`、`imu_postroll_s` 控制；`duration_s` 只表示图像采集窗口，不包含 IMU 前后滚、排空和校验。按传感器时间戳核验覆盖，不得用睡眠时长假定已有足够数据。
- IMU 非空原始 `header.frame_id` 优先；空值才使用配置 `imu_frame_id` 回退并告警。`ImuRecord.frame_id` 与 JSONL `frame_id` 始终保留原始字符串（包括空串），有效坐标系单独保存在 session 和内部 manifest，导出 bag 时才对空值应用回退。会话内有效坐标系变化必须失败，不得改写六轴数值或伪造原始坐标系。
- 相机与 IMU 在同一启动阶段的参照时间戳差超过 5 秒时拒绝 `camera-imu` 录制，不能把主动前滚造成的首张保存图像延后当成时钟偏移。
- 连续采集按完整双目对限频、分频和入队；任何丢弃都必须同时丢弃左右目。
- `--save-rate-hz` 与 `--frame-divisor N` 仅用于连续双目和 `camera-imu`，两者互斥。默认按 PTS 保存双目 5 Hz（约 200 ms 一对）；可显式使用源序号分频。IMU 收到的消息全部保存。
- PTS 限频以首对时间锚定目标时刻，选择到达目标的第一对，避免周期舍入累计导致边界多跳一帧。
- 采集先写 `.inprogress`，成功校验后提交正式目录；失败时保留为 `.failed`。

## 输出目录

录制命令的 `--output` 是输出根目录，默认 `./data`。每次录制自动生成：

```text
<output>/<dataset_name>_YYMMDD_hhmm/
```

目录名使用运行设备本地时间。前缀可由 CLI `--dataset-name` 或 INI `dataset_name` 指定；空 INI 值按模式选择 `stereo`、`stereo_imu` 或 `imu`。显式前缀为 1–64 个 ASCII 字母、数字、下划线或连字符，首字符是字母或数字。相同前缀同一分钟目录已存在时必须报错，包含 `.inprogress`、`.failed` 状态，不追加秒数或覆盖。实际会话目录及固定 `dataset_id` 写入命令输出与 session。

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
- INI 只允许 `[capture]` 段，必须含 `schema_version = 1.0.0`；不接受 `[DEFAULT]` 参数、其他段、重复键或未知键。值不加引号；注释单独以 `;` 或 `#` 起行。关闭插值，`%`、`${...}` 按字面处理。模板只列常用采集项，其他项沿用内置默认值；cali/vis 硬件清单保留在文档中，不设置 `hardware_layout` 段。
- 旧的自定义 YAML/JSON 配置需转换内容，不能只改扩展名。INI 运行配置与目录数据契约相互独立；数据 manifest 写入与校验不依赖 PyYAML。
- `script/reference_config.ini` 列出全部可加载配置项及 CLI 专用选项注释，保持与内置默认值一致；`default_config.ini` 保持精简。新配置项必须同步修改参考配置、数据类、CLI、会话元数据、文档和测试，常用项再加入精简模板。
- 不得静默改变左右目含义、时间单位、目录 schema 或默认话题。
- 根 `dataset.yaml` 必须使用字符串 `schema_version: "1.0.0"`、`type: kalibr_directory_dataset` 和 `dataset_id/cameras/imus`。相机必需 `id/timestamps/images`，附加 `frame_id/resolution/clock_source`；IMU 必需 `id/data`，附加有效 `frame_id/clock_source/angular_velocity_unit/linear_acceleration_unit`。topic 为可选元数据，新采集仅在 meta 中保留话题供 bag 导出；目录标定按 ID 关联。公共协议另允许根 `created_at/producer`。根文件仍用标准库 JSON 序列化，未知字段拒绝；原始 IMU frame ID 保留于 JSONL，有效 ID 不覆盖原值。
- 内部采集信息写入 `meta/capture_dataset.yaml`，使用同一字符串版本和 `kind: capture_dataset`。session 使用 `kind: capture_session`、`dataset_manifest: kalibr_directory_dataset`、`capture_layout: compact`。联合采集只写一份 `imu/imu0.csv`，表头为 `timestamp_ns,wx,wy,wz,ax,ay,az`，根直接引用；不交换轴、不变号、不缩放。纯双目不创建 IMU 文件，根使用 `imus: []`。
- Kalibr 不使用的辅助文件统一放 `meta/`，包括 `capture_dataset.yaml`、`pairs.csv`、`session.json`、`validation.json`；联合采集的 `imu_full.jsonl` 保留完整原消息，供 bag 导出，不是第二份标定 CSV。
- 历史采集 schema 1/2/3 和旧双 CSV 布局明确拒绝；新格式缺少 metadata 或 IMU CSV/JSONL 也是硬错误。核对 manifest、session、CSV/JSONL 的身份、单位、路径、时间戳和六轴内容，不能修改历史文件规避不兼容。
- 内部 manifest 记录 image_format、jpeg_quality 和 lossy，不使用数据校验和清单。JPEG 检查首尾标志并完整解码、核验单通道和尺寸，不保证发现所有可解码的像素损坏。bag 导出使用解码后的 mono8，不能恢复 JPEG 压缩前像素。
- `image_workers`／`--image-workers` 默认 2，允许 1–8，BMP/JPEG 共用一个复用线程池。每图一个任务，同时处理最多 `ceil(image_workers/2)` 对；协调线程按提交顺序成对提交文件和 CSV。
- 默认等待队列容量 2 对。缓冲池按需分配，容量为等待队列 + 同时处理对数 + 1 个生产者槽位；满时在映射前整对丢弃。成功入队才转移缓冲所有权，所有成功/失败任务结束后才能归还，禁止无限积压。
- CSV/IMU JSONL 每 0.5 秒批量刷新，正常停止和可处理异常退出时最终刷新，不逐帧 fsync。进程强杀或断电可能丢失尚未刷新的记录；后台刷新异常必须传播为录制失败。
- `write_performance` 保留每对从图像任务提交到有序提交完成的延迟统计；包含线程池/顺序等待，不包含输入等待队列、取帧或 fsync。多对重叠时累计延迟不是总录制时间。`capture_stages`、`image_stages`、`capture_stats.pipeline` 分别记录取帧/复制、编码/写入/元数据、排队/容量/排空；另记录峰值 RSS 和校验时间。统计只用固定大小累计计数器。
- 仅 `camera-imu`、`imu-continuous` 和 `probe --with-imu` 可以依赖 ROS 2 Python 环境。
- `imu-continuous` 为独立 Allan 原始 CSV 采集，不调用 AX API、不要求相机或图像编码库。只生成 `imu/imu0.csv` 与 `meta/session.json`、`meta/validation.json`，session 使用字符串版本 `1.0.0`、`kind: imu_capture_session`、`capture_layout: imu_allan_csv`。此模式不生成双目 Kalibr manifest，不支持现有 ROS1 导出。六轴使用原始值，CSV 与校验均流式处理；时间重复/回退、非有限值、有效 frame ID 变化或停流为失败。默认 60 秒、收到的消息全存，不使用图像限频或 IMU 前后滚。第一条消息后开始计时，Ctrl+C 可提前正常收尾；不足两条样本或异常保留 `.failed`。
- 写盘队列满时只丢弃完整双目对，并记录 AX 错误、源序号缺口、分频/限频跳过数及队列丢弃数。
- 独立 `imu-continuous` 另订阅 `imu_temperature_topic`（默认 `/imu/temperature`，sensor_msgs/msg/Temperature），其 CSV 为 `timestamp_ns,wx,wy,wz,ax,ay,az,temperature`，温度为摄氏度。按回调接收顺序沿用最新温度，不做按 header 时间戳匹配或插值；variance 不使用，允许 NaN，temperature 本身必须有限。首条温度前跳过 IMU 并统计预热数，首条可保存 IMU 后计时，等待超时失败。温度不更新时继续沿用，超过 2 秒告警；不改变 camera-imu 的七列 CSV。新校验器要求八列，不修改既有七列独立 IMU 数据。
- 源序号缺口必须在每次成功 VIN Get 后统计，包含重同步取得的帧；录制窗口在预热和 IMU 前滚之后建立新基线。同步淘汰按左右单帧独立计数，限频/分频跳过和写盘丢弃按双目对计数。保留全程 `sync_retries` 仅作诊断，不得从窗口缺口中相减；`successful_vin_get_v1` 是统计方法标识，不是文档版本。
- 不做磁盘容量或写速硬准入；4K 数据量只提示。

## 测试要求

从 `script` 目录执行：

```bash
python3 calibration_data.py self-test
python3 -m unittest discover -s tests -v
```

涉及 ABI、映射、同步、释放、分频或数据格式时，必须覆盖对应无硬件测试。安装 `rosbags==0.11.0` 后，ROS1 bag 生成与回读测试不得跳过。没有执行的可选依赖测试或板端测试必须明确说明。

目录契约改动还需用可用的真实 `kalibr_bag_io.directory.DirectoryReader` 读取小型 BMP/JPEG 测试数据，检查自定义话题、数量、解码、身份和原始 IMU 数值；缺少该 reader 时明确跳过。两工程联动按用户授权修改代码，不运行实际标定或改变历史数据来代替契约测试。

板端验收至少检查：

- 4K 与 1080p 分别执行 `probe`；
- 两侧为 NV12，尺寸与参数一致；
- PTS 严格递增，源帧率合理，左右差不超过 200 µs；
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
