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

本项目以 Python 采集双目标定、双目–IMU 外参和单路 USB 相机数据：

- 双目通过 `ctypes` 调用 AXERA VIN 接口；
- USB 相机通过 `ctypes` 调用既有 IVPS grp 3/chn 0，不创建或重配管线；
- IMU 通过 ROS 2 `sensor_msgs/msg/Imu` 订阅；
- 目录数据集是原始数据，ROS1 bag 仅在采集完成后离线生成；
- 不使用 `ros2 bag record`，也不负责启动、停止或重配相机流水线。

关键路径：

- 本地工程：`Q:\project\kalibr_cedarobo`
- 脚本：`Q:\project\kalibr_cedarobo\script`
- SDK 权威头文件：`D:\tool\mipi_tools\sensor_timing_check_tool\bsp_sdk\include`
- 当前 AX650 板端（无 Docker）：`/workspace/minsky/Vincent/calib_capture`

外部 SDK 和参考工程只读，不得修改。

## 硬件映射

| 模式 | 输出 | 左目 | 右目 | 接口 | 格式与分辨率 |
|---|---|---|---|---|---|
| cali | VIN | pipe 2 / chn 0 | pipe 0 / chn 0 | `AX_VIN_GetYuvFrame` | NV12；3840×2160 或 1920×1080 |
| vis | VIN | pipe 2 / chn 0 | pipe 0 / chn 0 | `AX_VIN_GetYuvFrame` | NV12，3840×2160 |
| vis | IVPS | grp 7 / chn 0 | grp 8 / chn 0 | `AX_IVPS_GetChnFrame` | YUV400，640×384 |
| usb | IVPS | grp 3 / chn 0 | — | `AX_IVPS_GetChnFrame` | 固定 1920×1080，落盘 mono8 |

双目正式标定只使用 cali VIN。不得将 vis 的 640×384 IVPS 图像用于双目标定录制。USB 命令只读 grp 3/chn 0 且严格校验 1920×1080，不提供分辨率覆盖。`--resolution` 只用于 VIN 命令校验上游实际输出，不得修改相机 config、重配 VIN/IVPS 或缩放图像。

## 采集契约

- 临时 `stereo-raw-triggered` 独立于正式 NV12 路径。固定 `AX_VIN_GetRawFrame(pipe, IFE=0, L=0, owner, timeout)`，用同一 owner/pipe/node/exposure 严格释放；无 chn 参数。默认左右 pipe 2/0。不自动设置 dump、RunOnce、SendRawFrame 或 AE，不切换相机服务。允许未压缩及 SDK LOSSLESS(1) Bayer RAW，保存 `u32FrameSize` 原字节（保留 padding，不解包、不转灰度），格式/stride/PTS 在 `meta/pairs.jsonl`，未知 Bayer 排列明确记录 unknown。每次 Enter 后先取放约 0.2 秒帧再采集，超时失败；不声称清队列等同硬件触发。输出前缀 stereo_raw；无 IMU、Kalibr manifest、bag 支持。验证完成前不集成到 stereo-triggered。只做文件长度与配对检查，不能宣称 RAW 像素已解码验证。

- `cam0` 始终是左目，`cam1` 始终是右目。
- `stereo-baselink-triggered` 固定按 14 个 base_link 姿态采集，不接受 `--poses`。左右图像使用相同姿态名；各自真实 PTS 必须继续保存在 `timestamps.csv`，不得用姿态名替代时间戳字段。
- `usb-continuous` 和 `usb-baselink-triggered` 是单相机模式，只创建 `cam0` 和 `meta/frames.csv`，不得复制同一图像伪造 `cam1`。后者沿用相同 14 姿态及姿态文件名；前者只用 PTS `--save-rate-hz`（别名 `--rate-hz`）限频。
- USB 成功取帧后必须使用同一 grp/chn 和同一 `AX_VIDEO_FRAME_T` 调用 `AX_IVPS_ReleaseChnFrame`，每帧严格释放一次。编码线程不得持有 IVPS 帧或映射。
- 默认左目 pipe 2/chn 0、右目 pipe 0/chn 0，CLI 可显式覆盖。
- 标定输入必须为 NV12，落盘默认无损 `mono8` BMP；可配置 `image_format = jpeg`、`jpeg_quality = 95` 保存有损灰度 JPG。质量范围 1–100；JPEG 只使用 OpenCV/NumPy，不使用 Pillow，不自动安装板端依赖，不缩放图像。
- 数据文件不计算 SHA-256，录制、结束收尾、离线校验和 bag 导出都只做结构与时序检查。BMP 按原 bottom-up 布局大块写入；不生成 `meta/checksums.sha256`。SDK 启动契约哈希不属于逐帧数据处理，仍保留。
- 使用 `u64PhyAddr[0]` 调用 `AX_SYS_MmapCache`；缓存失效后直接复制有效 Y 平面到 Python 自有缓冲区，再立即 Munmap 并释放 SDK 帧。BMP 在复制时形成 bottom-up 行序，JPEG 保持 top-down。不得解引用其他进程的 `u64VirAddr`，不得让 SDK 映射或帧进入编码线程。
- `AX_VIN_GetYuvFrame` 获取完整 `AX_IMG_INFO_T`，必须把同一个对象交回原 pipe/chn 的 `AX_VIN_ReleaseYuvFrame`。每个成功获取的帧只释放一次。
- 左右 PTS 差默认不超过 200 µs（含边界）；超限时仅重取较早侧，最多 10 次。
- 相机时间戳使用各自 `u64PTS × 1000` ns；IMU 使用原始 `header.stamp`。
- `stereo-imu` 默认在图像窗口前、后各保存至少 1 秒 IMU，分别由 `imu_preroll_s`、`imu_postroll_s` 控制；`duration_s` 只表示图像采集窗口，不包含 IMU 前后滚、排空和校验。按传感器时间戳核验覆盖，不得用睡眠时长假定已有足够数据。
- IMU 非空原始 `header.frame_id` 优先；空值才使用配置 `imu_frame_id` 回退并告警。`ImuRecord.frame_id` 与 JSONL `frame_id` 始终保留原始字符串（包括空串），有效坐标系单独保存在 session 和内部 manifest，导出 bag 时才对空值应用回退。会话内有效坐标系变化必须失败，不得改写六轴数值或伪造原始坐标系。
- 相机与 IMU 在同一启动阶段的参照时间戳差超过 5 秒时拒绝 `stereo-imu` 录制，不能把主动前滚造成的首张保存图像延后当成时钟偏移。
- 连续采集按完整双目对限频、分频和入队；任何丢弃都必须同时丢弃左右目。
- `--save-rate-hz` 与 `--frame-divisor N` 仅用于连续双目和 `stereo-imu`，两者互斥。默认按 PTS 保存双目 5 Hz（约 200 ms 一对）；可显式使用源序号分频。IMU 收到的消息全部保存。`usb-continuous` 仅用 `--save-rate-hz/--rate-hz`，不支持源序号分频。
- PTS 限频以首对时间锚定目标时刻，选择到达目标的第一对，避免周期舍入累计导致边界多跳一帧。
- 采集先写 `.inprogress`，成功校验后提交正式目录；失败时保留为 `.failed`。

## 输出目录

录制命令的 `--output` 是输出根目录，默认 `./data`。每次录制自动生成：

```text
<output>/<dataset_name>_YYMMDD_hhmm/
```

目录名使用运行设备本地时间。前缀可由 CLI `--dataset-name` 或 INI `dataset_name` 指定；空 INI 值按模式选择 `stereo`、`stereo_base_link`、`stereo_imu`、`usb`、`usb_baselink` 或 `imu`。显式前缀为 1–64 个 ASCII 字母、数字、下划线或连字符，首字符是字母或数字。相同前缀同一分钟目录已存在时必须报错，包含 `.inprogress`、`.failed` 状态，不追加秒数或覆盖。实际会话目录及固定 `dataset_id` 写入命令输出与 session。

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
- 仅 `stereo-imu`、`imu-continuous` 和 `probe --with-imu` 可以依赖 ROS 2 Python 环境；USB 两个命令不依赖 ROS 2。
- `imu-continuous` 为独立 IMU 原始诊断 CSV 采集，不调用 AX API、不要求相机或图像编码库。生成 `imu/imu0.csv`、`meta/imu_timestamp_anomalies.jsonl`、`meta/session.json` 和 `meta/validation.json`，session 使用字符串版本 `1.0.0`、`kind: imu_capture_session`、`capture_layout: imu_allan_csv`。此模式不生成双目 Kalibr manifest，不支持现有 ROS1 导出。六轴、原始 header stamp 和回调到达顺序保持不变；时间重复/回退不再中止，必须保留全部行、写异常前后记录并在 validation/session 中告警。含非递增时间戳的数据不得直接进入 Allan/Kalibr，分析工具仍应拒绝。非有限值、非正/越界时间戳、有效 frame ID 变化或停流仍为失败。默认 60 秒，不使用图像限频或 IMU 前后滚。第一条消息后开始计时，Ctrl+C 可提前正常收尾；不足两条样本或致命异常保留 `.failed`。`stereo-imu` 仍严格拒绝任何 IMU 时间重复/回退，不能把诊断采集的宽容策略扩展到标定数据。
- PC 容器工具统一放 `script/imu_utils_docker/`：构建配置和补丁放 `docker/`，分析脚本放 `allan/`；仅 allan 子目录只读挂载到 `/scripts`，Compose 命令从 docker 子目录执行。输入根为 `data/allan_variance/data/`（只读），输出根为 `data/allan_variance/imu_utils_output/`；板端同步时排除 `script/imu_utils_docker/`。ROS1 `imu_utils` 仅作为 PC 外部交叉分析，不扩展项目 `export-ros1` 对独立 IMU 会话的支持范围。必须只读原始会话，把派生 bag、TXT/YAML/PNG、日志和报告写入独立的 `data/allan_variance/imu_utils_output/<会话名>/`；不得给该 bag 写 `ros1_export_result` 或改动 v1.0.0 数据契约。派生消息保持 `timestamp_ns`、六轴、单位、轴序和有效 frame ID，Header stamp 等于 record time，无姿态时显式标记未知；不得从沿用温度值伪造温度消息。
- `imu_utils` 派生 bag 必须全量回读，核对 topic、类型、数量、时间和六轴；回放后六轴 `numData` 必须全部等于 bag 消息数。丢消息结果只能进入 `diagnostic_*`，不能作为最终参数；图像和报告只从验证通过的最终目录生成。上游 `gyr_w/acc_w` 是 bias instability 最低值，不得直接映射为 Kalibr bias random walk。
- 写盘队列满时只丢弃完整双目对，并记录 AX 错误、源序号缺口、分频/限频跳过数及队列丢弃数。
- 独立 `imu-continuous` 默认另订阅 `imu_temperature_topic`（默认 `/imu/temperature`，sensor_msgs/msg/Temperature），其 CSV 为 `timestamp_ns,wx,wy,wz,ax,ay,az,temperature`，温度为摄氏度。按回调接收顺序沿用最新温度，不做按 header 时间戳匹配或插值；variance 不使用，允许 NaN，temperature 本身必须有限。首条温度前跳过 IMU 并统计预热数，首条可保存 IMU 后计时，等待超时失败。温度不更新时继续沿用，超过 2 秒告警。设备不发布温度时可显式使用 `--ignore-temperature`（INI 为 `use_temperature = false`），不创建温度订阅、不等待温度，独立 CSV 使用七列 `timestamp_ns,wx,wy,wz,ax,ay,az`，session 明确记录 `temperature.enabled=false`；不得伪造温度值。不改变 stereo-imu 的七列 CSV。
- 源序号缺口必须在每次成功 VIN Get 后统计，包含重同步取得的帧；录制窗口在预热和 IMU 前滚之后建立新基线。同步淘汰按左右单帧独立计数，限频/分频跳过和写盘丢弃按双目对计数。保留全程 `sync_retries` 仅作诊断，不得从窗口缺口中相减；`successful_vin_get_v1` 是统计方法标识，不是文档版本。
- 不做磁盘容量或写速硬准入；4K 数据量只提示。

## 测试要求

PC `script/raw_to_png.py` 仅接受未压缩、非 packed 的 uint16 Bayer，默认 4K/RGGB/little-endian；
OpenCV 显式四字母 Bayer 常量，测试必须核对红蓝通道、大小端、行 padding、有效位对齐和 PNG 回读。
不做 SDK LOSSLESS 解压，关联 metadata 显示压缩时拒绝；原文件只读，PNG 不覆盖，PC 工具不改变采集格式。

`script/read_imx678_eeprom.py` 是独立、只读、仅依赖标准库和板端 `i2ctransfer` 的 EEPROM 工具。
字段表以模组定义为准，连续覆盖 `0x0000..0x012A` 共 299 字节，CRC_SUM32 位于 `0x0127`；
不得恢复旧版 `0x012F` CRC 或 crop/scale/mirror 字段。读取使用 16 位内部地址并分块完成，不自动扫描
I2C 总线、不写 EEPROM。板端实测确认校验为 `0x0000..0x0126` 的无符号逐字节累加和，
对 `2^32` 取模并以大端存储；显式 `--strict-checksum` 时不匹配返回失败。
双目 YAML 导出默认固定 `i2c-2=left`、`i2c-1=right`，分别对应 VIN pipe 2 和 pipe 0；允许 CLI 覆盖。分别生成 `left.yaml`
和 `right.yaml`，每个文件用命名字段保存完整身份、时间、尺寸、内参、RADTAN8 系数、标定/验证
误差和检测率。不生成 schema/kind/order、矩阵或重复向量；校验只保存 `crc: true/false`，不保存
算法和中间数值，也不保存 warnings。YAML 使用标准库生成、临时文件原子提交，默认不覆盖已有文件。

Allan 显式时间区间默认追加小时后缀，如 `_1h-24h`、`_1.5h-5.2h`；仅起点用 `2h-end`。整数无小数，其余 ROUND_HALF_UP 一位，不影响选区。完整 `--output` 覆盖命名；显示舍入同名仍须拒绝覆盖。用户明确授权重跑替换分析结果时可备份后替换，不涉及原始采集数据。

用户显式 `--kalibr-noise` 时允许 Allan 噪声拟合：-1/2 段求白噪声密度，+1/2 段求偏置随机游走（tau=3 s）。必须记录选段和误差，六轴齐全才导出候选标准 Kalibr YAML，三轴按最大值聚合；缺失不得以零、最低点或其他轴填补。未通过只保存诊断并返回非零，不得宣称候选等同于标定验收。默认无该选项仍不拟合。

Allan 时间选择 `--start-s/--end-s` 按第一条原始 IMU 时间戳起算，区间 `[start,end)`；先验证完整源数据，再切片，至少四条样本。报告保留选择范围和计数，统计仅针对选中段，不能静默删除段内样本或改动源文件。

Allan 默认输出固定为项目根目录下 `data/allan_variance/allantools_output/<输入目录名>/`，不随启动工作目录变化；显式 `--output` 指定完整结果目录。已有结果不覆盖，历史分析目录不迁移。

`allan` / `script/analyze_allan.py` 为 PC 离线分析入口，AllanTools、NumPy、Matplotlib 必须延迟导入，不影响板端采集。只读完整独立 IMU 会话，输出到单独目录，不改源数据；时间戳 int64、六轴 float64，温度只作摘要。非均匀采样默认拒绝，显式近似必须告警；不得静默删样、插值或自动宣称拟合出 Kalibr 噪声参数。缺少真实 AllanTools 时明确跳过引擎测试，不得将测试替身当成真实计算验证。

`script/analyze_imu_timeseries.py` 和 `script/analyze_imu_noise.py` 为 PC 离线诊断入口，严格读取独立 IMU 八列 CSV，先流式验证完整源数据，再加载选区；源文件只读，派生结果写入源会话同级 `analysis/<会话名>/`。时域曲线使用 min/max 包络降采样；噪声诊断仅逐轴减均值，不滤波、不去趋势、不插值或重采样。高斯性与白噪声必须分别报告工程条件和显著性检验，`pass/approximate/fail` 仅为诊断结论，不得宣称等同于 Kalibr 噪声模型验收。NumPy、Matplotlib、SciPy 仅为 PC 分析依赖，不得引入板端采集路径。

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
cd /workspace/minsky/Vincent/calib_capture
export BSP_MSP_DIR=/soc/lib
export LD_LIBRARY_PATH=/soc/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
export ROS_DOMAIN_ID=93
source /opt/ros/humble/setup.bash
```

当前板端直接运行，无 Docker 或宿主机目录映射。`/root/.bashrc` 已设置 `ROS_DOMAIN_ID=93` 并 source ROS 2，交互登录通常无需重复设置；非交互 SSH 命令必须显式提供等效环境。板端只同步 `script/`；不要同步 `doc/`、顶层 `AGENTS.md`、`data/` 或 `tmp/`。板端使用现有环境，不安装新包或库；`requirements-capture.txt` 中的 NumPy 仅用于可选加速。ROS1 bag 导出和 `rosbags` 依赖只在 PC 本机使用。

ROS1 imu_utils 外部工具按用户最新要求只读取八列 imu/imu0.csv，不要求或读取 meta/session.json、meta/validation.json；帧名由 --frame-id 指定（默认 imu_link），不声称恢复原始 frame ID。此项覆盖前述外部 bag 保持有效 frame ID 的默认要求，不改变采集端契约。已移除 test_workflow.py；保留 CSV 必要读取检查、bag 回读与分析输出检查。

imu_utils 每次运行输出顶层只放 bag、最终参数 YAML、三张 PNG 和 Markdown 报告；中间曲线 TXT、转换摘要、日志、验证、运行记录和源码补丁全部放 work/。验证与绘图命令的结果目录参数指向 work/。

外部 imu_utils / allan_variance_ros 的 CSV→bag 与分析入口支持 --start-s/--end-s 或 --start-h/--end-h，均相对原始 CSV 首条时间戳，区间 [start,end)。完整源数据先检查，bag 和计算只使用选中样本，时间戳不归零。统计针对选段，记录源总数和选区；两个引擎比较必须使用同一选段。默认输出带小时区间后缀，保留一位小数仅用于命名，已有结果禁止覆盖。
